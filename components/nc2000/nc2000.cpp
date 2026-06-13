#include "nc2000.h"
#include "comm.h"
#include "disassembler.h"
#include "ram.h"
#include "state.h"
#include "cpu.h"
#include "mem.h"
#include "io.h"
#include "rom.h"
#include "nor.h"
#include "nand.h"
#include <SDL2/SDL.h>
#include <SDL_timer.h>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include "sound.h"
#include "compare/c6502.h"
#include "console.h"
#include "iv_uart.h"
extern WqxRom nc2k_rom;

nc2k_states_t nc2k_states;
BusWrapper *dummy_bus = nullptr;

//static uint32_t& version = nc1020_states.version;

static uint8_t* keypad_matrix = nc2k_states.keypad_matrix;


void save_state(string file_name){
	if(file_name.empty()) file_name=nc2k_rom.statesPath;
	else file_name+=".state";
	// 打印保存时的关键状态
	printf("save_state(): sleeping=%d, lcdon=%d, lcden=%d, lcdbuffaddr=0x%x\n",
		nc2k_states.slept, nc2k_states.lcdon, nc2k_states.lcden, nc2k_states.lcdbuffaddr);
	/* Atomic save: write a complete .tmp first, then swap it in. A power-loss (the
	 * NC1020 has no clean shutdown) can then never truncate the LIVE state — it either
	 * survives whole or the previous good save stays. Writing in place used to leave a
	 * half-written .state on power-loss, which load_state would then read as garbage. */
	string tmp_name = file_name + ".tmp";
	FILE* file = fopen(tmp_name.c_str(), "wb");
	if (file == NULL) {
		printf("states tmp %s open failed, skip saving!\n", tmp_name.c_str());
		return;
	}
	int size = &nc2k_states.SAVE_STATE_END - &nc2k_states.SAVE_STATE_BEGIN;
	printf("saving %d bytes to state file...\n", size);
	size_t wrote = fwrite(&nc2k_states.SAVE_STATE_BEGIN, 1, size, file);
	fflush(file);
	fclose(file);
	if ((int)wrote != size) {                 /* short write -> keep the previous save */
		printf("state write short (%d of %d) — discarding tmp, previous save kept\n", (int)wrote, size);
		remove(tmp_name.c_str());
		return;
	}
	/* Swap the new save in. FAT rename() can't overwrite, so move the current save aside
	 * to .bak first; if the rename then fails we roll .bak back — so a failure can never
	 * lose BOTH saves (the previous bug: remove(dest) then a failed rename left nothing).
	 * On success drop .bak. If we crash mid-swap, the next boot's load_state recovers from
	 * .tmp/.bak. */
	string bak_name = file_name + ".bak";
	remove(bak_name.c_str());                        /* clear any stale backup        */
	rename(file_name.c_str(), bak_name.c_str());     /* current save -> .bak (ok if absent) */
	if (rename(tmp_name.c_str(), file_name.c_str()) != 0) {
		printf("state rename %s -> %s failed (errno=%d) — rolling back previous save\n",
			tmp_name.c_str(), file_name.c_str(), errno);
		rename(bak_name.c_str(), file_name.c_str()); /* restore the previous good save */
		remove(tmp_name.c_str());
		return;
	}
	remove(bak_name.c_str());                        /* success -> drop the backup    */
	printf("state saved to file %s!!\n",file_name.c_str());
}

void delete_state(string file_name){
	if(file_name.empty()) file_name=nc2k_rom.statesPath;
	else file_name+=".state";
	if(remove(file_name.c_str())==0){
		printf("state file %s deleted!\n",file_name.c_str());
	}else{
		printf("state file %s not exist or no permission to delete!\n",file_name.c_str());
	}
}

bool load_state(){
	printf("load_state() called with path = %s\n", nc2k_rom.statesPath.c_str());
	// 先保存默认状态用于对比
	bool def_slept = nc2k_states.slept;
	bool def_lcdon = nc2k_states.lcdon;
	bool def_lcden = nc2k_states.lcden;
	uint16_t def_lcdbuffaddr = nc2k_states.lcdbuffaddr;
	
	/* Recover from an interrupted save: the canonical .state can be missing only if a
	 * crash hit between the .bak/.tmp rename steps in save_state — and in that window a
	 * COMPLETE copy still exists. Prefer the newest: .tmp (new save, fully written but not
	 * yet swapped in) then .bak (previous good save). The ret==size check below still
	 * guards against any partial file, so recovery can't load garbage. */
	string spath = nc2k_rom.statesPath;
	FILE* file = fopen(spath.c_str(), "rb");
	if (file == NULL) {
		string tmp = spath + ".tmp", bak = spath + ".bak";
		if ((file = fopen(tmp.c_str(), "rb")) != NULL)
			printf("primary .state missing — recovering from %s\n", tmp.c_str());
		else if ((file = fopen(bak.c_str(), "rb")) != NULL)
			printf("primary .state missing — recovering from %s\n", bak.c_str());
		if (file == NULL) {
			printf("no state file (.state/.tmp/.bak, errno=%d) — skip loading!\n", errno);
			return false;
		}
	}
	int size = &nc2k_states.SAVE_STATE_END - &nc2k_states.SAVE_STATE_BEGIN;
	printf("about to read %d bytes from state file...\n", size);
	int ret=fread(&nc2k_states.SAVE_STATE_BEGIN, 1, size, file);
	fclose(file);
	printf("loaded states from %s, ret=%d (expected %d)\n", nc2k_rom.statesPath.c_str(), ret, size);
	printf("loaded state values: sleeping=%d, lcdon=%d, lcden=%d, lcdbuffaddr=0x%x\n",
		nc2k_states.slept, nc2k_states.lcdon, nc2k_states.lcden, nc2k_states.lcdbuffaddr);
	// 强制重置关键状态
	nc2k_states.slept = false;
	nc2k_states.pending_wake_up = false;
	nc2k_states.lcdon = 1;
	nc2k_states.lcden = 1;
	nc2k_states.lcdbuffaddr = 0x09c0;
	nc2k_states.lcdbuffaddrmask = 0x0fff;
	/* The state is saved at clk-off (idle standby): ram_io[0x05]>>5==7. If we resume
	 * from the saved PC without clearing those clock-stop bits the firmware stays
	 * parked in standby and never repaints (black screen). Clear them so the firmware
	 * resumes its main loop and repaints the saved session in place — no cpu->reset()
	 * (a reset re-inits the UI back to the home/"出厂" screen, losing the session). */
	nc2k_states.ram_io[0x05] &= 0x1f;
	printf("fixed state values: sleeping=%d, lcdon=%d, lcden=%d, lcdbuffaddr=0x%x clk05=0x%02x\n",
		nc2k_states.slept, nc2k_states.lcdon, nc2k_states.lcden, nc2k_states.lcdbuffaddr, nc2k_states.ram_io[0x05]);
	super_switch();
	/* Reject a TRUNCATED state (power-loss during a previous save, or a layout change
	 * from a firmware update): a partial blob is a half-old/half-new inconsistent state
	 * that may NOT crash — so the boot-strike crash-loop guard never fires and the user
	 * is stuck with silent corruption. ret==size means a full, matching state; anything
	 * else -> treat as no state (caller cold-boots into a clean session). */
	if (ret != size) {
		printf("state file truncated/mismatched (got %d of %d) — ignoring, cold boot\n", ret, size);
		return false;
	}
	return true;
}

void LoadNC2k(){
	dummy_bus= new BusWrapper();

	init_io(); //for old io implemet only
	
	void CreateHotlinkMapping();
	CreateHotlinkMapping();

	init_nor();
	if(pc1000mode||nc1020mode) {
		init_rom();
	}
	if(nc2000mode||nc3000mode) {
		read_nand0_file();
		read_nand_file();
	}

	init_mem();

	if(nc1020mode){
		ram_io[0x0b]=0x01;
	}

	//reset_cpu_states();
	initalize_illegal_op_tables();
	init_cpu_new();

	if(nc2000mode||nc3000mode){
		//nc3000c-lee has it but seems like no need?
		//ram_io[0x18]=0x20;
	}

	bool state_loaded = false;
	if(enable_load_state){
		state_loaded = load_state();
		if(nc2000mode){
			void sync_time_2000();
			if(enable_auto_time_sync) sync_time_2000();
		}
		if(nc1020mode){
			void sync_time_1020();
			if(enable_auto_time_sync) sync_time_1020();
		}
	}
	if (!state_loaded) {
		nc2k_states.init(); // Init if state not loaded
	}

	super_switch();

	if(enable_load_state && !state_loaded){
		/* Cold boot (no saved state): nc2k_states.init() above zeroed the CPU, so
		 * its PC is 0, not the firmware reset vector. Do a cold reset so the firmware
		 * starts from its reset vector, initialises (资料整理) and boots. Without this
		 * it executes from PC=0 and never paints -> black screen / "没正常启动". */
		void cold_reset();
		cold_reset();
	} else if(enable_load_state && reset_after_load_state){
		void set_warm_reset_flag();
		set_warm_reset_flag();
		void warm_reset();
		warm_reset();
	}
}

static unsigned short &lcdbuffaddr = nc2k_states.lcdbuffaddr;
static unsigned short &lcdbuffaddrmask = nc2k_states.lcdbuffaddrmask;
bool is_grey_mode(){
	if(console_on) return false;

	unsigned short lcd_addr = lcdbuffaddr&lcdbuffaddrmask;
	//printf("lcdaddr=%x\n",lcd_addr);
	//fflush(stdout);
	if(nc2000mode||nc3000mode||nc1020mode)
		return lcd_addr==0x1380;
	return false;
}
bool CopyLcdBuffer(uint8_t* buffer){
    unsigned short lcd_addr = lcdbuffaddr&lcdbuffaddrmask;
	if (lcd_addr == 0) return false;

	if(nc1020mode){
		if(!is_grey_mode()){
			memcpy(buffer, ram_buff + lcd_addr, 1600 );
		}else{
			memcpy(buffer, ram_buff + lcd_addr, 1600 *2);
		}
		return true;;
	}
	else if(nc2000mode||nc3000mode){
		if(!is_grey_mode()){
			//TODO: cannot use lcd_addr, it has some offset
			//// 应该是lcdaddr io哪里没有模拟好。 如果lcd end addr不是(1fff,0fff,...)可能应该忽略lcd_addr的设置
			memcpy(buffer, ram_buff + 0x19c0, 1600 );
		}else{
			memcpy(buffer, ram_buff + lcd_addr, 1600 *2);
		}
		return true;
	}else{
		memcpy(buffer, ram_buff + lcd_addr, 1600);
		return true;
	}
	assert(false);
}

void RunTimeSlice(uint32_t time_slice) {
	uint32_t new_cycles = time_slice * CYCLES_MS;

	if(!fast_forward) {
		new_cycles= new_cycles * speed_multiplier;
	}else if(fast_forward_limit==0){
		new_cycles= new_cycles*1;
	}else{
		new_cycles= new_cycles * fast_forward_limit;
	}

	u64_t target_cycles=nc2k_states.cycles +new_cycles;

	while (nc2k_states.cycles < target_cycles) {
		if(cpu_loop_version == CPU_RUN1){
			cpu_run();
		}else if (cpu_loop_version == CPU_RUN2){
			cpu_run2();
		}else if (cpu_loop_version == CPU_RUN3){
			cpu_run3();
		}else{
			assert(false);
		}
		post_cpu_run_sound_handling();
	}
}

void save_flash(string file){
	write_nand0_file(file);
	write_nand_file(file);
	SaveNor(file);
	printf("flash saved to file!!\n");
}

void nc2k_state_warm_reset(){

	uint8_t* ioReg=nc2k_states.ram_io;

	const int simple_warm_reset=true;
  if(simple_warm_reset){
	ioReg[0x05] &=0x1f;  //reset cks
	nc2k_states.speed_scaledown=1;
  } else {
    //0x00
    ioReg[0x00]=0;

    //0x01
    nc2k_states.inner_interrupt_control&=0xfc; //TMBIE TMAIE clear
    
    //0x02
    ////ioReg[0x02]=0;          //if reset both 0x02 and 0x03, on/off key will trigger cold reset
    
    //0x03
    ////ioReg[0x03]=0;

    //0x04
    nc2k_states.w04_b03_TBC &=0xf0;
    
    //0x05
    ioReg[0x05] &=0x1f;
    ioReg[0x05] &=0xf7;
    nc2k_states.lcdon=0;
    nc2k_states.speed_scaledown=1;

    //0x0a
    ioReg[0x0a] &=0xe0;
    
    //0x0b
    ioReg[0x0b]&=0xfd;
    nc2k_states.lcden=0;

    //0x0c
    ioReg[0x0c]&=0xfc;
    nc2k_states.w0c_b67_TMODESL =0;
    nc2k_states.w0c_b45_TM0S =0;
    nc2k_states.w0c_b23_TM1S = 0;
    nc2k_states.w0c_b345_TMS = 0;

    //0x0d
    ioReg[0x0d]&=0xf8;

    //0x14
    ioReg[0x14]=0;

    //0x19
    ioReg[0x19]=0xef;

    //0x1a
    ioReg[0x1a]=0;

    //0x1b
    ioReg[0x1b]=0;

    //0x1c
    ioReg[0x1c]&=0xbf;

    //0x1e
    ioReg[0x1e]=0;
  }

    super_switch();
}

void nc2k_state_cold_reset(){
    nc2k_state_warm_reset();
    clear_iv();//if this is put into warm_reset, alarm wakeup will not work correcly

    //memset(ram_io,0,sizeof(nc2k_states.ram_io));
    //memset(ext_reg, 0, sizeof(nc2k_states.ext_reg));
    nc2k_states.reset();
    super_switch();
}
