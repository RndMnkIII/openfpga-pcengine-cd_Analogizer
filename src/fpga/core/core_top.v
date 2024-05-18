//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

//`default_nettype none

module core_top (

    //
    // physical connections
    //

    ///////////////////////////////////////////////////
    // clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

    input wire clk_74a,  // mainclk1
    input wire clk_74b,  // mainclk1 

    ///////////////////////////////////////////////////
    // cartridge interface
    // switches between 3.3v and 5v mechanically
    // output enable for multibit translators controlled by pic32

    // GBA AD[15:8]
    inout  wire [7:0] cart_tran_bank2,
    output wire       cart_tran_bank2_dir,

    // GBA AD[7:0]
    inout  wire [7:0] cart_tran_bank3,
    output wire       cart_tran_bank3_dir,

    // GBA A[23:16]
    inout  wire [7:0] cart_tran_bank1,
    output wire       cart_tran_bank1_dir,

    // GBA [7] PHI#
    // GBA [6] WR#
    // GBA [5] RD#
    // GBA [4] CS1#/CS#
    //     [3:0] unwired
    inout  wire [7:4] cart_tran_bank0,
    output wire       cart_tran_bank0_dir,

    // GBA CS2#/RES#
    inout  wire cart_tran_pin30,
    output wire cart_tran_pin30_dir,
    // when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
    // the goal is that when unconfigured, the FPGA weak pullups won't interfere.
    // thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
    // and general IO drive this pin.
    output wire cart_pin30_pwroff_reset,

    // GBA IRQ/DRQ
    inout  wire cart_tran_pin31,
    output wire cart_tran_pin31_dir,

    // infrared
    input  wire port_ir_rx,
    output wire port_ir_tx,
    output wire port_ir_rx_disable,

    // GBA link port
    inout  wire port_tran_si,
    output wire port_tran_si_dir,
    inout  wire port_tran_so,
    output wire port_tran_so_dir,
    inout  wire port_tran_sck,
    output wire port_tran_sck_dir,
    inout  wire port_tran_sd,
    output wire port_tran_sd_dir,

    ///////////////////////////////////////////////////
    // cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

    output wire [21:16] cram0_a,
    inout  wire [ 15:0] cram0_dq,
    input  wire         cram0_wait,
    output wire         cram0_clk,
    output wire         cram0_adv_n,
    output wire         cram0_cre,
    output wire         cram0_ce0_n,
    output wire         cram0_ce1_n,
    output wire         cram0_oe_n,
    output wire         cram0_we_n,
    output wire         cram0_ub_n,
    output wire         cram0_lb_n,

    output wire [21:16] cram1_a,
    inout  wire [ 15:0] cram1_dq,
    input  wire         cram1_wait,
    output wire         cram1_clk,
    output wire         cram1_adv_n,
    output wire         cram1_cre,
    output wire         cram1_ce0_n,
    output wire         cram1_ce1_n,
    output wire         cram1_oe_n,
    output wire         cram1_we_n,
    output wire         cram1_ub_n,
    output wire         cram1_lb_n,

    ///////////////////////////////////////////////////
    // sdram, 512mbit 16bit

    output wire [12:0] dram_a,
    output wire [ 1:0] dram_ba,
    inout  wire [15:0] dram_dq,
    output wire [ 1:0] dram_dqm,
    output wire        dram_clk,
    output wire        dram_cke,
    output wire        dram_ras_n,
    output wire        dram_cas_n,
    output wire        dram_we_n,

    ///////////////////////////////////////////////////
    // sram, 1mbit 16bit

    output wire [16:0] sram_a,
    inout  wire [15:0] sram_dq,
    output wire        sram_oe_n,
    output wire        sram_we_n,
    output wire        sram_ub_n,
    output wire        sram_lb_n,

    ///////////////////////////////////////////////////
    // vblank driven by dock for sync in a certain mode

    input wire vblank,

    ///////////////////////////////////////////////////
    // i/o to 6515D breakout usb uart

    output wire dbg_tx,
    input  wire dbg_rx,

    ///////////////////////////////////////////////////
    // i/o pads near jtag connector user can solder to

    output wire user1,
    input  wire user2,

    ///////////////////////////////////////////////////
    // RFU internal i2c bus 

    inout  wire aux_sda,
    output wire aux_scl,

    ///////////////////////////////////////////////////
    // RFU, do not use
    output wire vpll_feed,


    //
    // logical connections
    //

    ///////////////////////////////////////////////////
    // video, audio output to scaler
    output wire [23:0] video_rgb,
    output wire        video_rgb_clock,
    output wire        video_rgb_clock_90,
    output wire        video_de,
    output wire        video_skip,
    output wire        video_vs,
    output wire        video_hs,

    output wire audio_mclk,
    input  wire audio_adc,
    output wire audio_dac,
    output wire audio_lrck,

    ///////////////////////////////////////////////////
    // bridge bus connection
    // synchronous to clk_74a
    output wire        bridge_endian_little,
    input  wire [31:0] bridge_addr,
    input  wire        bridge_rd,
    output reg  [31:0] bridge_rd_data,
    input  wire        bridge_wr,
    input  wire [31:0] bridge_wr_data,

    ///////////////////////////////////////////////////
    // controller data
    // 
    // key bitmap:
    //   [0]    dpad_up
    //   [1]    dpad_down
    //   [2]    dpad_left
    //   [3]    dpad_right
    //   [4]    face_a
    //   [5]    face_b
    //   [6]    face_x
    //   [7]    face_y
    //   [8]    trig_l1
    //   [9]    trig_r1
    //   [10]   trig_l2
    //   [11]   trig_r2
    //   [12]   trig_l3
    //   [13]   trig_r3
    //   [14]   face_select
    //   [15]   face_start
    // joy values - unsigned
    //   [ 7: 0] lstick_x
    //   [15: 8] lstick_y
    //   [23:16] rstick_x
    //   [31:24] rstick_y
    // trigger values - unsigned
    //   [ 7: 0] ltrig
    //   [15: 8] rtrig
    //
    input wire [15:0] cont1_key,
    input wire [15:0] cont2_key,
    input wire [15:0] cont3_key,
    input wire [15:0] cont4_key,
    input wire [31:0] cont1_joy,
    input wire [31:0] cont2_joy,
    input wire [31:0] cont3_joy,
    input wire [31:0] cont4_joy,
    input wire [15:0] cont1_trig,
    input wire [15:0] cont2_trig,
    input wire [15:0] cont3_trig,
    input wire [15:0] cont4_trig

);

  // not using the IR port, so turn off both the LED, and
  // disable the receive circuit to save power
  assign port_ir_tx              = 0;
  assign port_ir_rx_disable      = 1;

  // bridge endianness
  assign bridge_endian_little    = 0;

  // cart is unused, so set all level translators accordingly
  // directions are 0:IN, 1:OUT
//  assign cart_tran_bank3 = 8'hzz;            // these pins are not used, make them inputs
//  assign cart_tran_bank3_dir = 1'b0;
 
//  assign cart_tran_bank2 = 8'hzz;            // these pins are not used, make them inputs
//  assign cart_tran_bank2_dir = 1'b0;
//  assign cart_tran_bank1 = 8'hzz;            // these pins are not used, make them inputs
//  assign cart_tran_bank1_dir = 1'b0;
 
//  assign cart_tran_bank0 = {1'b0, TXDATA, LED, 1'b0};    // LED and TXD hook up here
//  assign cart_tran_bank0_dir = 1'b1;
 
//  assign cart_tran_pin30 = 1'bz;            // this pin is not used, make it an input
//  assign cart_tran_pin30_dir = 1'b0;
//  assign cart_pin30_pwroff_reset = 1'b1;    
 
//  assign cart_tran_pin31 = 1'bz;            // this pin is an input
//  assign cart_tran_pin31_dir = 1'b0;        // input
 // UART
 wire TXDATA;                        // your UART transmit data hooks up here
 //wire RXDATA = cart_tran_pin31;        // your UART RX data shows up here
 
 // button/LED
 wire LED = 0;                    // LED hooks up here.  HIGH = light up, LOW = off
 //wire BUTTON = cart_tran_bank3[0];    // button data comes out here.  LOW = pressed, HIGH = unpressed

  // link port is input only
  assign port_tran_so            = TXDATA;
  assign port_tran_so_dir        = 1'b1;  // SO is output only
  wire RXDATA = port_tran_si;
  assign port_tran_si_dir        = 1'b0;  // SI is input only
  assign port_tran_sck           = 1'bz;
  assign port_tran_sck_dir       = 1'b0;  // clock direction can change
  assign port_tran_sd            = 1'bz;
  assign port_tran_sd_dir        = 1'b0;  // SD is input and not used

  // tie off the rest of the pins we are not using
//  assign cram0_a                 = 'h0;
//  assign cram0_dq                = {16{1'bZ}};
//  assign cram0_clk               = 0;
//  assign cram0_adv_n             = 1;
//  assign cram0_cre               = 0;
//  assign cram0_ce0_n             = 1;
//  assign cram0_ce1_n             = 1;
//  assign cram0_oe_n              = 1;
//  assign cram0_we_n              = 1;
//  assign cram0_ub_n              = 1;
//  assign cram0_lb_n              = 1;
//
//  assign cram1_a                 = 'h0;
//  assign cram1_dq                = {16{1'bZ}};
//  assign cram1_clk               = 0;
//  assign cram1_adv_n             = 1;
//  assign cram1_cre               = 0;
//  assign cram1_ce0_n             = 1;
//  assign cram1_ce1_n             = 1;
//  assign cram1_oe_n              = 1;
//  assign cram1_we_n              = 1;
//  assign cram1_ub_n              = 1;
//  assign cram1_lb_n              = 1;

  // assign dram_a                  = 'h0;
  // assign dram_ba                 = 'h0;
  // assign dram_dq                 = {16{1'bZ}};
  // assign dram_dqm                = 'h0;
  // assign dram_clk                = 'h0;
  // assign dram_cke                = 'h0;
  // assign dram_ras_n              = 'h1;
  // assign dram_cas_n              = 'h1;
  // assign dram_we_n               = 'h1;

//  assign sram_a                  = 'h0;
//  assign sram_dq                 = {16{1'bZ}};
//  assign sram_oe_n               = 1;
//  assign sram_we_n               = 1;
//  assign sram_ub_n               = 1;
//  assign sram_lb_n               = 1;

	wire   [31:0]				CORE_OUTPUT;
	wire   [31:0]				CORE_INPUT;

  assign dbg_tx                  = 1'bZ;
  assign user1                   = 1'bZ;
  assign aux_scl                 = 1'bZ;
  assign vpll_feed               = 1'bZ;

	wire [31:0] 	mpu_reg_bridge_rd_data;
	wire [31:0] 	mpu_ram_bridge_rd_data;
	wire [2:0]		video_slot_output;
  // for bridge write data, we just broadcast it to all bus devices
  // for bridge read data, we have to mux it
  // add your own devices here
  always @(*) begin
    casex (bridge_addr)
      default: begin
        bridge_rd_data <= 0;
      end
      32'h2xxxxxxx: begin
        bridge_rd_data <= sd_read_data;
      end
      32'hF7000000: begin 
        bridge_rd_data <= {18'h0,analogizer_settings};
        //bridge_rd_data <= analogizer_settings;
      end
		  32'h8xxxxxxx: begin
        bridge_rd_data <= mpu_ram_bridge_rd_data;
      end
		  32'hf0xxxxxx: begin
        bridge_rd_data <= mpu_reg_bridge_rd_data;
      end
      32'hF8xxxxxx: begin
        bridge_rd_data <= cmd_bridge_rd_data;
      end
    endcase
  end

  always @(posedge clk_74a) begin
    if (reset_delay > 0) begin
      reset_delay <= reset_delay - 1;
    end

    if (bridge_wr) begin
      casex (bridge_addr)
//        32'h0: begin
//          ioctl_download <= bridge_wr_data[0];
//        end
//        32'h4: begin
//          save_download <= bridge_wr_data[0];
//        end
//        32'h8: begin
//          is_sgx <= bridge_wr_data[0];
//        end
        32'h50: begin
          reset_delay <= 32'h100000;
        end
        32'h100: begin
          turbo_tap_enable <= bridge_wr_data[0];
        end
        32'h104: begin
          button6_enable <= bridge_wr_data[0];
        end
        32'h108: begin
          button1_turbo_speed <= bridge_wr_data[1:0];
        end
        32'h10C: begin
          button2_turbo_speed <= bridge_wr_data[1:0];
        end
        32'h200: begin
          overscan_enable <= bridge_wr_data[0];
        end
        32'h204: begin
          extra_sprites_enable <= bridge_wr_data[0];
        end
        // 32'h208: begin
        //   raw_rgb_enable <= bridge_wr_data[0];
//        end
        32'h300: begin
          master_audio_boost <= bridge_wr_data[1:0];
        end
        32'h304: begin
          adpcm_audio_boost <= bridge_wr_data[0];
        end
        32'h308: begin
          cd_audio_boost <= bridge_wr_data[0];
        end
        /*[ANALOGIZER_HOOK_BEGIN]*/
				32'hF7000000: analogizer_settings  <=  bridge_wr_data[13:0];
				/*[ANALOGIZER_HOOK_END]*/
      endcase
    end
  end

  //
  // host/target command handler
  //
  wire reset_n;  // driven by host commands, can be used as core-wide reset
  wire [31:0] cmd_bridge_rd_data;

  // bridge host commands
  // synchronous to clk_74a
  wire status_boot_done = pll_core_locked;
  wire status_setup_done = pll_core_locked;  // rising edge triggers a target command
  wire status_running = reset_n;  // we are running as soon as reset_n goes high

	wire            dataslot_requestread;
	wire    [15:0]  dataslot_requestread_id;
	wire            dataslot_requestread_ack = 1;
	wire            dataslot_requestread_ok = 1;

	wire            dataslot_requestwrite;
	wire    [15:0]  dataslot_requestwrite_id;
	wire    [31:0]  dataslot_requestwrite_size;
	wire            dataslot_requestwrite_ack = 1;
	wire            dataslot_requestwrite_ok = 1;

	wire            dataslot_update;
	wire    [15:0]  dataslot_update_id;
	wire    [31:0]  dataslot_update_size;

	wire            dataslot_allcomplete;

  wire savestate_supported;
  wire [31:0] savestate_addr;
  wire [31:0] savestate_size;
  wire [31:0] savestate_maxloadsize;

  wire savestate_start;
  wire savestate_start_ack;
  wire savestate_start_busy;
  wire savestate_start_ok;
  wire savestate_start_err;

  wire savestate_load;
  wire savestate_load_ack;
  wire savestate_load_busy;
  wire savestate_load_ok;
  wire savestate_load_err;

  wire osnotify_inmenu;
  
	wire     [31:0] rtc_epoch_seconds;
	wire     [31:0] rtc_date_bcd;
	wire     [31:0] rtc_time_bcd;
	wire            rtc_valid;
  
  
	wire            target_dataslot_read;       
	wire            target_dataslot_write;

	wire            target_dataslot_ack;        
	wire            target_dataslot_done;
	wire    [2:0]   target_dataslot_err;

	wire     [15:0] target_dataslot_id;
	wire     [31:0] target_dataslot_slotoffset;
	wire     [31:0] target_dataslot_bridgeaddr;
	wire     [31:0] target_dataslot_length;

  // bridge target commands
  // synchronous to clk_74a
  wire  [35:0] EXT_BUS;

  // bridge data slot access

  wire [9:0]  datatable_addr;
  wire 		  datatable_wren;
  
  wire        datatable_rden;
  wire [31:0] datatable_data;
  wire [31:0] datatable_q;

core_bridge_cmd icb (

    .clk                    		( clk_74a ),
    .reset_n                		( reset_n ),
	 .clk_sys					 		( clk_74a ),

    .bridge_endian_little   		( bridge_endian_little ),
    .bridge_addr            		( bridge_addr ),
    .bridge_rd              		( bridge_rd ),
    .bridge_rd_data         		( cmd_bridge_rd_data ),
    .bridge_wr              		( bridge_wr ),
    .bridge_wr_data         		( bridge_wr_data ),
    
    .status_boot_done       		( status_boot_done ),
    .status_setup_done      		( status_setup_done ),
    .status_running         		( status_running ),

    .dataslot_requestread       ( dataslot_requestread ),
    .dataslot_requestread_id    ( dataslot_requestread_id ),
    .dataslot_requestread_ack   ( dataslot_requestread_ack ),
    .dataslot_requestread_ok    ( dataslot_requestread_ok ),

    .dataslot_requestwrite      ( dataslot_requestwrite ),
    .dataslot_requestwrite_id   ( dataslot_requestwrite_id ),
    .dataslot_requestwrite_size ( dataslot_requestwrite_size ),
    .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack ),
    .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok ),

    .dataslot_update            ( dataslot_update ),
    .dataslot_update_id         ( dataslot_update_id ),
    .dataslot_update_size       ( dataslot_update_size ),
    
    .dataslot_allcomplete   		( dataslot_allcomplete ),

    .rtc_epoch_seconds      		( rtc_epoch_seconds ),
    .rtc_date_bcd           		( rtc_date_bcd ),
    .rtc_time_bcd           		( rtc_time_bcd ),
    .rtc_valid              		( rtc_valid ),
    
    .savestate_supported    		( savestate_supported ),
    .savestate_addr         		( savestate_addr ),
    .savestate_size         		( savestate_size ),
    .savestate_maxloadsize  		( savestate_maxloadsize ),

    .savestate_start        		( savestate_start ),
    .savestate_start_ack    		( savestate_start_ack ),
    .savestate_start_busy   		( savestate_start_busy ),
    .savestate_start_ok     		( savestate_start_ok ),
    .savestate_start_err    		( savestate_start_err ),

    .savestate_load         ( savestate_load ),
    .savestate_load_ack     ( savestate_load_ack ),
    .savestate_load_busy    ( savestate_load_busy ),
    .savestate_load_ok      ( savestate_load_ok ),
    .savestate_load_err     ( savestate_load_err ),

    .osnotify_inmenu        ( osnotify_inmenu ),
    
    .target_dataslot_read       ( target_dataslot_read ),
    .target_dataslot_write      ( target_dataslot_write ),

    .target_dataslot_ack        ( target_dataslot_ack ),
    .target_dataslot_done       ( target_dataslot_done ),
    .target_dataslot_err        ( target_dataslot_err ),

    .target_dataslot_id         ( target_dataslot_id ),
    .target_dataslot_slotoffset ( target_dataslot_slotoffset ),
    .target_dataslot_bridgeaddr ( target_dataslot_bridgeaddr ),
    .target_dataslot_length     ( target_dataslot_length ),

    .datatable_addr         ( datatable_addr ),
    .datatable_wren         ( datatable_wren ),
    .datatable_rden         ( datatable_rden ),
    .datatable_data         ( datatable_data ),
    .datatable_q            ( datatable_q )

);

  reg ioctl_download = 0;
  reg save_download = 0;
  reg is_sgx = 0;
  wire ioctl_wr;
  wire [23:0] ioctl_addr;
  wire [15:0] ioctl_dout;

always @(posedge clk_74a) begin
  ioctl_download 	<= dataslot_requestwrite_id == 1 	|| dataslot_requestread_id == 1;
  save_download 	<= dataslot_requestwrite_id == 2 	|| dataslot_requestread_id == 2;
end

  wire [31:0] sd_read_data;

  wire sd_rd;
  wire sd_wr;
  wire [15:0] sd_buff_din;
  wire [15:0] sd_buff_dout;

  wire [24:0] sd_buff_addr = sd_wr ? sd_buff_addr_in : sd_buff_addr_out;
  wire [24:0] sd_buff_addr_in;
  wire [24:0] sd_buff_addr_out;



  wire ioctl_download_s;
  wire save_download_s;
  wire is_sgx_s;

  synch_3 #(
      .WIDTH(3)
  ) download_s (
      {ioctl_download, save_download, is_sgx},
      {ioctl_download_s, save_download_s, is_sgx_s},
      clk_mem_85_91
  );


  data_loader #(
      .ADDRESS_MASK_UPPER_4(4'h1),
      .ADDRESS_SIZE(24),
      .OUTPUT_WORD_SIZE(2),

      .WRITE_MEM_CLOCK_DELAY(32),
      .WRITE_MEM_EN_CYCLE_LENGTH(16)
  ) data_loader (
      .clk_74a(clk_74a),
      .clk_memory(clk_mem_85_91),

      .bridge_wr(bridge_wr),
      .bridge_endian_little(bridge_endian_little),
      .bridge_addr(bridge_addr),
      .bridge_wr_data(bridge_wr_data),

      .write_en  (ioctl_wr),
      .write_addr(ioctl_addr),
      .write_data(ioctl_dout)
  );

  data_loader #(
      .ADDRESS_MASK_UPPER_4(4'h2),
      .ADDRESS_SIZE(24),
      .OUTPUT_WORD_SIZE(2),

      .WRITE_MEM_CLOCK_DELAY(16),
      .WRITE_MEM_EN_CYCLE_LENGTH(8)
  ) save_data_loader (
      .clk_74a(clk_74a),
      .clk_memory(clk_sys_42_95),

      .bridge_wr(bridge_wr),
      .bridge_endian_little(bridge_endian_little),
      .bridge_addr(bridge_addr),
      .bridge_wr_data(bridge_wr_data),

      .write_en  (sd_wr),
      .write_addr(sd_buff_addr_in),
      .write_data(sd_buff_dout)
  );

  data_unloader #(
      .ADDRESS_MASK_UPPER_4(4'h2),
      .ADDRESS_SIZE(25),
      .INPUT_WORD_SIZE(2),

      .READ_MEM_CLOCK_DELAY(16)
  ) save_data_unloader (
      .clk_74a(clk_74a),
      .clk_memory(clk_sys_42_95),

      .bridge_rd(bridge_rd),
      .bridge_endian_little(bridge_endian_little),
      .bridge_addr(bridge_addr),
      .bridge_rd_data(sd_read_data),

      .read_en  (sd_rd),
      .read_addr(sd_buff_addr_out),
      .read_data(sd_buff_din)
  );

  wire [15:0] cont1_key_s;
  wire [15:0] cont2_key_s;
  wire [15:0] cont3_key_s;
  wire [15:0] cont4_key_s;

  synch_3 #(
      .WIDTH(16)
  ) cont1_s (
      cont1_key,
      cont1_key_s,
      clk_sys_42_95
  );

  synch_3 #(
      .WIDTH(16)
  ) cont2_s (
      cont2_key,
      cont2_key_s,
      clk_sys_42_95
  );

  synch_3 #(
      .WIDTH(16)
  ) cont3_s (
      cont3_key,
      cont3_key_s,
      clk_sys_42_95
  );

  synch_3 #(
      .WIDTH(16)
  ) cont4_s (
      cont4_key,
      cont4_key_s,
      clk_sys_42_95
  );

  // Settings

  reg turbo_tap_enable = 0;
  reg button6_enable = 0;
  reg [1:0] button1_turbo_speed = 0;
  reg [1:0] button2_turbo_speed = 0;

  reg overscan_enable = 0;
  reg extra_sprites_enable = 0;
  reg raw_rgb_enable = 0;
  reg mb128_enable = 0;

  reg cd_audio_boost = 0;
  reg adpcm_audio_boost = 0;
  reg [1:0] master_audio_boost = 0;

  reg [31:0] reset_delay = 0;

  // Sync

  wire turbo_tap_enable_s;
  wire button6_enable_s;
  wire [1:0] button1_turbo_speed_s;
  wire [1:0] button2_turbo_speed_s;

  wire overscan_enable_s;
  wire extra_sprites_enable_s;
  wire raw_rgb_enable_s;
  wire mb128_enable_s;

  wire cd_audio_boost_s;
  wire adpcm_audio_boost_s;
  wire [1:0] master_audio_boost_s;

  synch_3 #(
      .WIDTH(14)
  ) settings_s (
      {
        turbo_tap_enable,
        button6_enable,
        button1_turbo_speed,
        button2_turbo_speed,
        overscan_enable,
        extra_sprites_enable,
        raw_rgb_enable,
        mb128_enable,
        cd_audio_boost,
        adpcm_audio_boost,
        master_audio_boost
      },
      {
        turbo_tap_enable_s,
        button6_enable_s,
        button1_turbo_speed_s,
        button2_turbo_speed_s,
        overscan_enable_s,
        extra_sprites_enable_s,
        raw_rgb_enable_s,
        mb128_enable_s,
        cd_audio_boost_s,
        adpcm_audio_boost_s,
        master_audio_boost_s
      },
      clk_sys_42_95
  );

  wire [15:0] audio_l;
  wire [15:0] audio_r;

  wire [1:0] dotclock_divider;
  wire border;

/*[ANALOGIZER_HOOK_BEGIN]*/
  //Pocket Menu settings
  reg [13:0] analogizer_settings = 0;
  wire [13:0] analogizer_settings_s;

  synch_3 #(.WIDTH(14)) sync_analogizer({analogizer_settings}, {analogizer_settings_s}, clk_sys_42_95);

  always @(*) begin
    snac_game_cont_type   = analogizer_settings_s[4:0];
    snac_cont_assignment  = analogizer_settings_s[9:6];
    analogizer_video_type = analogizer_settings_s[13:10];
  end

  //*** Analogizer Interface V1.1 ***
  reg analogizer_ena;
  reg [3:0] analogizer_video_type;
  reg [4:0] snac_game_cont_type /* synthesis keep */;
  reg [3:0] snac_cont_assignment /* synthesis keep */;
  
  //switch between Analogizer SNAC and Pocket Controls for P1-P4 (P3,P4 when uses PCEngine Multitap)
  wire [15:0] p1_btn, p2_btn, p3_btn, p4_btn;
  reg [15:0] p1_controls, p2_controls, p3_controls, p4_controls;

  always @(posedge clk_sys_42_95) begin
    if(snac_game_cont_type == 5'h0) begin //SNAC is disabled
                  p1_controls <= cont1_key_s;
                  p2_controls <= cont2_key_s;
                  p3_controls <= cont3_key_s;
                  p4_controls <= cont4_key_s;
    end
    else begin
      case(snac_cont_assignment)
      4'h0:    begin 
                  p1_controls <= p1_btn;
                  p2_controls <= cont2_key_s;
                  p3_controls <= cont3_key_s;
                  p4_controls <= cont4_key_s;
                end
      4'h1:    begin 
                  p1_controls <= cont1_key_s;
                  p2_controls <= p1_btn;
                  p3_controls <= cont3_key_s;
                  p4_controls <= cont4_key_s;
                end
      4'h2:    begin
                  p1_controls <= p1_btn;
                  p2_controls <= p2_btn;
                  p3_controls <= cont3_key_s;
                  p4_controls <= cont4_key_s;
                end
      4'h3:    begin
                  p1_controls <= p2_btn;
                  p2_controls <= p1_btn;
                  p3_controls <= cont3_key_s;
                  p4_controls <= cont4_key_s;
                end
      4'h4:    begin
                  p1_controls <= p1_btn;
                  p2_controls <= p2_btn;
                  p3_controls <= p3_btn;
                  p4_controls <= p4_btn;
                end
      4'h5:    begin
                  p1_controls <= p4_btn;
                  p2_controls <= p3_btn;
                  p3_controls <= p2_btn;
                  p4_controls <= p1_btn;
                end
      4'h6:    begin
                  p1_controls <= cont1_key_s;
                  p2_controls <= cont2_key_s;
                  p3_controls <= p1_btn;
                  p4_controls <= p2_btn;
                end
      default: begin
                  p1_controls <= cont1_key_s;
                  p2_controls <= cont2_key_s;
                  p3_controls <= cont3_key_s;
                  p4_controls <= cont4_key_s;
                end
      endcase
    end
  end

  //wire clk_vid = ce_pix; //video_rgb_clock; //Fixed one bit shift error on RGB channels.

  wire SYNC = ~^{video_hs_core, video_vs_core};
  wire  ANALOGIZER_DE = ~(h_blank || v_blank);

  //create aditional switch to blank Pocket screen.
  wire [23:0] video_rgb_pocket;
  assign video_rgb_pocket = (analogizer_video_type[3]) ? 24'h000000: vid_rgb_core;

    //Video synchronizer for Analogizer DAC
    // reg ce_pix_r;
    // reg vsync_r, hsync_r, csync_r;
    // reg hblank_r, vblank_r, blank_r;
    // reg [23:0] rgb_color_r;
    // always @(posedge clk_sys_42_95) begin
    //     ce_pix_r <= ce_pix;
        
    //     if (!ce_pix_r && ce_pix) begin //rising edge
    //         vsync_r <= video_vs_core;
    //         hsync_r <= video_hs_core;
    //         csync_r <= SYNC;
    //         hblank_r <= h_blank;
    //         vblank_r <= v_blank;
    //         blank_r <= ANALOGIZER_DE;
    //         rgb_color_r <= vid_rgb_core;
    //     end
    // end


// SET PAL and NTSC TIMING and pass through status bits. ** YC must be enabled in the qsf file **
wire [39:0] CHROMA_PHASE_INC;
wire [26:0] COLORBURST_RANGE;
wire [4:0] CHROMA_ADD;
wire [4:0] CHROMA_MULT;
//wire PALFLAG;

	parameter NTSC_REF = 3.579545;   
	parameter PAL_REF = 4.43361875;
	// Colorburst Lenth Calculation to send to Y/C Module, based on the CLK_VIDEO of the core
	localparam [6:0] COLORBURST_START = (3.7 * (CLK_VIDEO_NTSC/NTSC_REF));
	localparam [9:0] COLORBURST_NTSC_END = (9 * (CLK_VIDEO_NTSC/NTSC_REF)) + COLORBURST_START;
	localparam [9:0] COLORBURST_PAL_END = (10 * (CLK_VIDEO_PAL/PAL_REF)) + COLORBURST_START;
 
	// Parameters to be modifed
  parameter CLK_VIDEO_NTSC = 42.954545; // Must be filled E.g XX.X Hz - CLK_VIDEO
	parameter CLK_VIDEO_PAL = 42.954545; // Must be filled E.g XX.X Hz - CLK_VIDEO
  //PAL CLOCK FREQUENCY SHOULD BE 42.56274
	localparam [39:0] NTSC_PHASE_INC = 40'd91625968981; //d91_625_958_315; //d91_625_968_981; // ((NTSC_REF**2^40) / CLK_VIDEO_NTSC) - SNES Example;
	//localparam [39:0] PAL_PHASE_INC = 40'd114532461227; // ((PAL_REF*2^40) / CLK_VIDEO_PAL)- SNES Example;

	// Send Parameters to Y/C Module
	assign CHROMA_PHASE_INC = NTSC_PHASE_INC; 
	//assign PALFLAG = (analogizer_video_type == 4'h4) || (analogizer_video_type == 4'hC); 
  assign CHROMA_ADD = 5'd0; //yc_chroma_add_s;
  assign CHROMA_MULT = 5'd0; //yc_chroma_mult_s;
 	assign COLORBURST_RANGE = {COLORBURST_START, COLORBURST_NTSC_END, COLORBURST_PAL_END}; // Pass colorburst length

//42_954_545
openFPGA_Pocket_Analogizer #(.MASTER_CLK_FREQ(42_954_545)) analogizer (
	.i_clk(clk_sys_42_95),
	.i_rst(~reset_n || reset_delay > 0), //i_rst is active high
	.i_ena(1'b1),
	//Video interface
	.analog_video_type(analogizer_video_type),
  // .R(rgb_color_r[23:16]),
	// .G(rgb_color_r[15:8]),
	// .B(rgb_color_r[7:0]),
  // .Hblank(hblank_r),
  // .Vblank(vblank_r),
  // .BLANKn(blank_r),
  // .Csync(csync_r), //composite SYNC on HSync.
  // .Hsync(hsync_r),
	// .Vsync(vsync_r),
  .R(vid_rgb_core[23:16]),
	.G(vid_rgb_core[15:8]),
	.B(vid_rgb_core[7:0]),
  .Hblank(h_blank),
  .Vblank(v_blank),
  .BLANKn(ANALOGIZER_DE),
  .Csync(SYNC), //composite SYNC on HSync.
  .Hsync(video_hs_core),
	.Vsync(video_vs_core),
	.video_clk(clk_sys_42_95),
  //Video Y/C Encoder interface
  .PALFLAG(1'b0),
  //.CVBS(yc_cvbs_s),
	.MULFLAG(1'b0),
	.CHROMA_ADD(CHROMA_ADD),
	.CHROMA_MULT(CHROMA_MULT),
	.CHROMA_PHASE_INC(CHROMA_PHASE_INC),
	.COLORBURST_RANGE(COLORBURST_RANGE),
  //Video SVGA Scandoubler interface
  .ce_divider(3'd7), //div4
	//SNAC interface
	.conf_AB((snac_game_cont_type >= 5'd16)),              //0 conf. A(default), 1 conf. B (see graph above)
	.game_cont_type(snac_game_cont_type), //0-15 Conf. A, 16-31 Conf. B
	.p1_btn_state(p1_btn),
	.p2_btn_state(p2_btn),  
  .p3_btn_state(p3_btn),
	.p4_btn_state(p4_btn),  
	//Pocket Analogizer IO interface to the Pocket cartridge port
	.cart_tran_bank2(cart_tran_bank2),
	.cart_tran_bank2_dir(cart_tran_bank2_dir),
	.cart_tran_bank3(cart_tran_bank3),
	.cart_tran_bank3_dir(cart_tran_bank3_dir),
	.cart_tran_bank1(cart_tran_bank1),
	.cart_tran_bank1_dir(cart_tran_bank1_dir),
	.cart_tran_bank0(cart_tran_bank0),
	.cart_tran_bank0_dir(cart_tran_bank0_dir),
	.cart_tran_pin30(cart_tran_pin30),
	.cart_tran_pin30_dir(cart_tran_pin30_dir),
	.cart_pin30_pwroff_reset(cart_pin30_pwroff_reset),
	.cart_tran_pin31(cart_tran_pin31),
	.cart_tran_pin31_dir(cart_tran_pin31_dir),
	//debug
	.o_stb()
);
/*[ANALOGIZER_HOOK_END]*/



  pce pce (
      .clk_sys_42_95(clk_sys_42_95),
      .clk_mem_85_91(clk_mem_85_91),

      .core_reset(~reset_mpu_l),
      .pll_core_locked(pll_core_locked),

      .sgx(CORE_OUTPUT[0]),
		.AC_EN(CORE_OUTPUT[1]),

      // Input
      .p1_button_1(p1_controls[4]),
      .p1_button_2(p1_controls[5]),
      .p1_button_3(p1_controls[6]),
      .p1_button_4(p1_controls[7]),
      .p1_button_5(p1_controls[8]),
      .p1_button_6(p1_controls[9]),
      .p1_button_select(p1_controls[14]),
      .p1_button_start(p1_controls[15]),
      .p1_dpad_up(p1_controls[0]),
      .p1_dpad_down(p1_controls[1]),
      .p1_dpad_left(p1_controls[2]),
      .p1_dpad_right(p1_controls[3]),

      .p2_button_1(p2_controls[4]),
      .p2_button_2(p2_controls[5]),
      .p2_button_3(p2_controls[6]),
      .p2_button_4(p2_controls[7]),
      .p2_button_5(p2_controls[8]),
      .p2_button_6(p2_controls[9]),
      .p2_button_select(p2_controls[14]),
      .p2_button_start(p2_controls[15]),
      .p2_dpad_up(p2_controls[0]),
      .p2_dpad_down(p2_controls[1]),
      .p2_dpad_left(p2_controls[2]),
      .p2_dpad_right(p2_controls[3]),

      .p3_button_1(p3_controls[4]),
      .p3_button_2(p3_controls[5]),
      .p3_button_3(p3_controls[6]),
      .p3_button_4(p3_controls[7]),
      .p3_button_5(p3_controls[8]),
      .p3_button_6(p3_controls[9]),
      .p3_button_select(p3_controls[14]),
      .p3_button_start(p3_controls[15]),
      .p3_dpad_up(p3_controls[0]),
      .p3_dpad_down(p3_controls[1]),
      .p3_dpad_left(p3_controls[2]),
      .p3_dpad_right(p3_controls[3]),

      .p4_button_1(p4_controls[4]),
      .p4_button_2(p4_controls[5]),
      .p4_button_3(p4_controls[6]),
      .p4_button_4(p4_controls[7]),
      .p4_button_5(p4_controls[8]),
      .p4_button_6(p4_controls[9]),
      .p4_button_select(p4_controls[14]),
      .p4_button_start(p4_controls[15]),
      .p4_dpad_up(p4_controls[0]),
      .p4_dpad_down(p4_controls[1]),
      .p4_dpad_left(p4_controls[2]),
      .p4_dpad_right(p4_controls[3]),

      // Settings
      .turbo_tap_enable(turbo_tap_enable_s),
      .button6_enable(button6_enable_s),
      .button1_turbo_speed(button1_turbo_speed_s),
      .button2_turbo_speed(button2_turbo_speed_s),

      .overscan_enable(overscan_enable_s),
      .extra_sprites_enable(extra_sprites_enable_s),
      .raw_rgb_enable(1'b1),

      .mb128_enable(mb128_enable_s),

      .cd_audio_boost(cd_audio_boost_s),
      .adpcm_audio_boost(adpcm_audio_boost_s),
      .master_audio_boost(master_audio_boost_s),

      // Data in
      .ioctl_wr(ioctl_wr),
      .ioctl_addr(ioctl_addr),
      .ioctl_dout(ioctl_dout),
      .cart_download(ioctl_download_s),

      // Data out
      .sd_wr(sd_wr),
      .sd_buff_addr(sd_buff_addr[8:1]),
      .sd_lba(sd_buff_addr[24:9]),
      .sd_buff_dout(sd_buff_dout),
      .sd_buff_din(sd_buff_din),
      .save_download(save_download_s),

      // SDRAM
      .dram_a						(dram_a),
      .dram_ba						(dram_ba),
      .dram_dq						(dram_dq),
      .dram_dqm					(dram_dqm),
      .dram_clk					(dram_clk),
      .dram_cke					(dram_cke),
      .dram_ras_n					(dram_ras_n),
      .dram_cas_n					(dram_cas_n),
      .dram_we_n					(dram_we_n),
		
		//SRAM
		.sram_a                 ( sram_a ),
		.sram_dq                ( sram_dq ),
		.sram_oe_n              ( sram_oe_n ),
		.sram_we_n              ( sram_we_n ),
		.sram_ub_n              ( sram_ub_n ),
		.sram_lb_n              ( sram_lb_n ),
		
		//CRAM
		.cram0_a                ( cram0_a ),
		.cram0_dq               ( cram0_dq ),
		.cram0_wait             ( cram0_wait ),
		.cram0_clk              ( cram0_clk ),
		.cram0_adv_n            ( cram0_adv_n ),
		.cram0_cre              ( cram0_cre ),
		.cram0_ce0_n            ( cram0_ce0_n ),
		.cram0_ce1_n            ( cram0_ce1_n ),
		.cram0_oe_n             ( cram0_oe_n ),
		.cram0_we_n             ( cram0_we_n ),
		.cram0_ub_n             ( cram0_ub_n ),
		.cram0_lb_n             ( cram0_lb_n ),
		
//		.cram1_a                ( cram1_a ),
//		.cram1_dq               ( cram1_dq ),
//		.cram1_wait             ( cram1_wait ),
//		.cram1_clk              ( cram1_clk ),
//		.cram1_adv_n            ( cram1_adv_n ),
//		.cram1_cre              ( cram1_cre ),
//		.cram1_ce0_n            ( cram1_ce0_n ),
//		.cram1_ce1_n            ( cram1_ce1_n ),
//		.cram1_oe_n             ( cram1_oe_n ),
//		.cram1_we_n             ( cram1_we_n ),
//		.cram1_ub_n             ( cram1_ub_n ),
//		.cram1_lb_n             ( cram1_lb_n ),

      .ce_pix (ce_pix),
      .hblank (h_blank),
      .vblank (v_blank),
      .hsync  (video_hs_core),
      .vsync  (video_vs_core),
      .video_r(vid_rgb_core[23:16]),
      .video_g(vid_rgb_core[15:8]),
      .video_b(vid_rgb_core[7:0]),

      .dotclock_divider(dotclock_divider),
      .border(border),

      .audio_l(audio_l),
      .audio_r(audio_r),
		.EXT_BUS(EXT_BUS)
  );
  
  
wire reset_mpu_l;
wire IO_OSD;

substitute_mcu_apf_mister substitute_mcu_apf_mister(
		// Controls for the MPU
		.clk_mpu								( clk_74a ), 							// Clock of the MPU itself
		.clk_sys								( clk_sys_42_95 ),
		.clk_74a								( clk_74a ),							// Clock of the APF Bus
		.reset_n								( status_running),							// Reset from the APF System
		.reset_out							( reset_mpu_l ),						// Able to restart the core from the MPU if required
		
		// APF Bus controll
		.bridge_addr            		( bridge_addr ),
		.bridge_rd              		( bridge_rd ),
		.mpu_reg_bridge_rd_data       ( mpu_reg_bridge_rd_data ),		// Used for interactions
		.mpu_ram_bridge_rd_data       ( mpu_ram_bridge_rd_data ),		// Used for ram up/download
		.bridge_wr              		( bridge_wr ),
		.bridge_wr_data         		( bridge_wr_data ),
	  
	   // Debugging to the Cart	
		.rxd									( RXDATA ),
		.txd									( TXDATA ),
		
		// APF Controller access if required
		
		.cont1_key          				( cont1_key ),
		.cont2_key          				( cont2_key ),
		.cont3_key          				( cont3_key ),
		.cont4_key          				( cont4_key ),
		.cont1_joy          				( cont1_joy ),
		.cont2_joy          				( cont2_joy ),
		.cont3_joy          				( cont3_joy ),
		.cont4_joy          				( cont4_joy ),
		.cont1_trig         				( cont1_trig ),
		.cont2_trig         				( cont2_trig ),
		.cont3_trig         				( cont3_trig ),
		.cont4_trig         				( cont4_trig ),
		
		.cram_a                			( cram1_a ),
		.cram_dq               			( cram1_dq ),
		.cram_wait             			( cram1_wait ),
		.cram_clk              			( cram1_clk ),
		.cram_adv_n            			( cram1_adv_n ),
		.cram_cre              			( cram1_cre ),
		.cram_ce0_n            			( cram1_ce0_n ),
		.cram_ce1_n            			( cram1_ce1_n ),
		.cram_oe_n             			( cram1_oe_n ),
		.cram_we_n             			( cram1_we_n ),
		.cram_ub_n             			( cram1_ub_n ),
		.cram_lb_n             			( cram1_lb_n ),
		
		// MPU Controlls to the APF
		
		.dataslot_update            	( dataslot_update ),
		.dataslot_update_id         	( dataslot_update_id ),
		.dataslot_update_size       	( dataslot_update_size ),
	  
		.target_dataslot_read       	( target_dataslot_read ),
		.target_dataslot_write      	( target_dataslot_write ),

		.target_dataslot_ack        	( target_dataslot_ack ),
		.target_dataslot_done       	( target_dataslot_done ),
		.target_dataslot_err        	( target_dataslot_err ),

		.target_dataslot_id         	( target_dataslot_id ),
		.target_dataslot_slotoffset 	( target_dataslot_slotoffset ),
		.target_dataslot_bridgeaddr 	( target_dataslot_bridgeaddr ),
		.target_dataslot_length     	( target_dataslot_length ),

		.datatable_addr         		( datatable_addr ),
		.datatable_wren         		( datatable_wren ),
		.datatable_rden         		( datatable_rden ),
		.datatable_data         		( datatable_data ),
		.datatable_q            		( datatable_q ),
		
		// Core interactions
		.IO_UIO       						( EXT_BUS[34] ),
		.IO_OSD      						( EXT_BUS[35] ),
		.IO_STROBE    						( EXT_BUS[33] ),
		.IO_WAIT      						( EXT_BUS[32] ),
		.IO_DIN       						( EXT_BUS[15:0] ),
		.IO_DOUT      						( EXT_BUS[31:16] ),
		.IO_WIDE								( 1'b1 ),
		.CORE_OUTPUT						(CORE_OUTPUT),
		.CORE_INPUT							(CORE_INPUT)
	 
	 );
	 

  ////////////////////////////////////////////////////////////////////////////////////////

  // Video
  wire ce_pix;
  wire h_blank;
  wire v_blank;
  wire video_hs_core;
  wire video_vs_core;
  wire [23:0] vid_rgb_core;
  
  
  wire video_hs_wire;
  wire video_vs_wire;
  wire video_de_wire;
  wire [23:0] video_rgb_wire;

  assign video_rgb_clock = clk_sys_42_95;
  assign video_rgb_clock_90 = clk_vid_42_95_90deg;

  assign video_skip = 0;

  linebuffer linebuffer (
      .clk_vid(clk_sys_42_95),

      .vsync_in(video_vs_core),
      .hsync_in(video_hs_core),

      .ce_pix(ce_pix),
      .disable_pix(border),
      .rgb_in(vid_rgb_core),

      .vsync_out(video_vs_wire),
      .hsync_out(video_hs_wire),
		.slot		(CORE_OUTPUT[10:8]),
      .de(video_de_wire),
      .rgb_out(video_rgb_wire)
  );
  
  osd osd (
	.clk_sys		(clk_sys_42_95),
	.io_osd		(EXT_BUS[35]),
	.io_strobe	(EXT_BUS[33]),
	.io_din		(EXT_BUS[31:16]),

	.clk_video	(clk_sys_42_95),
	.din			(video_rgb_wire),

	.de_in		(video_de_wire),
	.vs_in		(video_vs_wire),
	.hs_in		(video_hs_wire),
	
	.dout			(video_rgb),
	.de_out		(video_de),
	.vs_out		(video_vs),
	.hs_out		(video_hs)
  
  );

  ///////////////////////////////////////////////

  sound_i2s #(
      .CHANNEL_WIDTH(16),
      .SIGNED_INPUT (1)
  ) sound_i2s (
      .clk_74a  (clk_74a),
      .clk_audio(clk_sys_42_95),

      .audio_l(audio_l),
      .audio_r(audio_r),

      .audio_mclk(audio_mclk),
      .audio_lrck(audio_lrck),
      .audio_dac (audio_dac)
  );

  ///////////////////////////////////////////////


  wire clk_mem_85_91;
  wire clk_sys_42_95;
  wire clk_vid_42_95_90deg;

  wire pll_core_locked;

  mf_pllbase mp1 (
      .refclk(clk_74a),
      .rst   (0),

      .outclk_0(clk_mem_85_91),
      .outclk_1(clk_sys_42_95),
      .outclk_2(clk_vid_42_95_90deg),

      .locked(pll_core_locked)
  );



endmodule
