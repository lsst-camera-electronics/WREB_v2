----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    19:01:52 02/19/2016 
-- Design Name: 
-- Module Name:    WREB_v2_top - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use work.max_11046_top_package.all;


-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

-- LSST libraries and packages 
--use work.RcmSciPackage.RcmSci;

entity WREB_v2_top is

  port (
    ------ Clock signals ------
    -- PGP serdes clk
    PgpRefClk_P : in std_logic;
    PgpRefClk_M : in std_logic;
    -- sys clock (this is a clk gnerate by a quartz on the board)
--    sysclk_p    : in std_logic;
--    sysclk_m    : in std_logic;

    ------ PGP signals ------
    PgpRx_p : in  std_logic;
    PgpRx_m : in  std_logic;
    PgpTx_p : out std_logic;
    PgpTx_m : out std_logic;

    -- CCD ADC
    adc_data_ccd_1 : in  std_logic_vector(15 downto 0);
    adc_cnv_ccd_1  : out std_logic;
    adc_sck_ccd_1  : out std_logic;
    adc_buff_pd    : out std_logic;

    -- ASPIC signals
    ASPIC_r_up_ccd_1_p   : out std_logic;
    ASPIC_r_up_ccd_1_n   : out std_logic;
    ASPIC_r_down_ccd_1_p : out std_logic;
    ASPIC_r_down_ccd_1_n : out std_logic;
    ASPIC_clamp_ccd_1_p  : out std_logic;
    ASPIC_clamp_ccd_1_n  : out std_logic;
    ASPIC_reset_ccd_1_p  : out std_logic;
    ASPIC_reset_ccd_1_n  : out std_logic;

    -- CCD Clocks signals
    par_clk_ccd_1_p    : out std_logic_vector(3 downto 0);
    par_clk_ccd_1_n    : out std_logic_vector(3 downto 0);
    ser_clk_ccd_1_p    : out std_logic_vector(2 downto 0);
    ser_clk_ccd_1_n    : out std_logic_vector(2 downto 0);
    reset_gate_ccd_1_p : out std_logic;
    reset_gate_ccd_1_n : out std_logic;

    ---- ASICs SPI link ---- 
    -- common signals
    asic_spi_mosi   : out std_logic;
    asic_spi_sclk   : out std_logic;
    asic_spi_miso_t : in  std_logic;
    asic_spi_miso_b : in  std_logic;

    -- ASPIC control signals
    ASPIC_ss_t_ccd_1 : out std_logic;
    ASPIC_ss_b_ccd_1 : out std_logic;
    ASPIC_spi_reset  : out std_logic;
    ASPIC_nap_ccd_1  : out std_logic;

--      ASPIC_time_ccd_1_p              : out std_logic;
--      ASPIC_time_ccd_1_n              : out std_logic;

    backbias_clamp : out std_logic;
    backbias_ssbe  : out std_logic;

    -- CABAC pulse
    pulse_ccd_1_p : out std_logic;
    pulse_ccd_1_n : out std_logic;


    ------ REB V & I sensors ------  
    LTC2945_SCL : inout std_logic;
    LTC2945_SDA : inout std_logic;

    ------ Temperature ------
-- DREB PCB temperature
    DREB_temp_sda : inout std_logic;
    DREB_temp_scl : inout std_logic;

-- CCD temperatures
    csb_24ADC  : out std_logic;
    sclk_24ADC : out std_logic;
    din_24ADC  : out std_logic;
    dout_24ADC : in  std_logic;

-- ASICs temp slow ADC and mux
    Tmux_ena_ccd_1     : out   std_logic;
    Tmux_a0_ccd_1      : out   std_logic;
    Tmux_a1_ccd_1      : out   std_logic;
    Temp_adc_scl_ccd_1 : inout std_logic;
    Temp_adc_sda_ccd_1 : inout std_logic;

    ------ DACs ------
-- cabac clock rails DAC
    ldac_RAILS      : out std_logic;
    din_RAILS       : out std_logic;
    sclk_RAILS      : out std_logic;
    sync_RAILS_dac0 : out std_logic;
    sync_RAILS_dac1 : out std_logic;

-- CCD current source
    ldac_CSGATE : out std_logic;
    din_CSGATE  : out std_logic;
    sync_CSGATE : out std_logic;
    sclk_CSGATE : out std_logic;

-- CABAC BIAS
    ldac_C_BIAS : out std_logic;
    din_C_BIAS  : out std_logic;
    sync_C_BIAS : out std_logic;
    sclk_C_BIAS : out std_logic;

-- max 11056 bias slow adc
    bs_adc_EOC               : in    std_logic;
    bs_adc_data_from_adc_dcr : inout std_logic_vector(3 downto 0);
    bs_adc_data_from_adc     : in    std_logic_vector(15 downto 4);
    bs_adc_CS                : out   std_logic;
    bs_adc_RD                : out   std_logic;
    bs_adc_WR                : out   std_logic;
    bs_adc_CONVST            : out   std_logic;
    bs_adc_SHDN              : out   std_logic;


-- max 11056 bias slow adc
    ck_adc_EOC               : in    std_logic;
    ck_adc_data_from_adc_dcr : inout std_logic_vector(3 downto 0);
    ck_adc_data_from_adc     : in    std_logic_vector(15 downto 4);
    ck_adc_CS                : out   std_logic;
    ck_adc_RD                : out   std_logic;
    ck_adc_WR                : out   std_logic;
    ck_adc_CONVST            : out   std_logic;
    ck_adc_SHDN              : out   std_logic;





--    ------ High speed ADCs ------
--    -- ADC for MUX0             
----      or_mux0                 : in  std_logic;                        
--    dco_mux0   : in  std_logic;
--    d_mux0     : in  std_logic_vector(11 downto 0);
--    -- ADC for MUX1
----      or_mux1                 : in  std_logic;                        
--    dco_mux1   : in  std_logic;
--    d_mux1     : in  std_logic_vector(11 downto 0);
--    -- common signals
----      cs_l_mux                        : out std_logic; 
--    pd_adc_mux : out std_logic;
---- sclk_mux                     : out std_logic;        
----      sdio_mux                        : out std_logic;
--    -- differential clock for high speed ADC (100 MHz)
--    clk_mux_p  : out std_logic;
--    clk_mux_n  : out std_logic;

    ------ MISC ------
-- Resistors
--    r_add : in std_logic_vector(7 downto 0);
    r_add : in std_logic_vector(6 downto 0);

-- Test port
    TEST : out std_logic_vector(3 downto 0);

-- Test led
    TEST_LED : out std_logic_vector(5 downto 0);

-- Power ON reset
    Pwron_Rst_L : in std_logic;

-- sync for power supply
    PWR_SYNC : out std_logic;

-- CCD clocks enable
    ccd_clk_en_out_p : out std_logic;
    ccd_clk_en_out_n : out std_logic;

-- ASPIC reference power down
    ASPIC_ref_sd : out std_logic;
    ASPIC_5V_sd  : out std_logic;

    -- GPIO
    gpio_0_p   : out std_logic;
    gpio_0_n   : out std_logic;
    gpio_0_dir : out std_logic;
    gpio_1_p   : out std_logic;
    gpio_1_n   : out std_logic;
    gpio_1_dir : out std_logic;

-- jitter studies signals
--      pll_recov_clk_p : in std_logic;
--      pll_recov_clk_n : in std_logic;

--      recov_clk_p                     : out std_logic;
--      recov_clk_n                     : out std_logic;

-- power supply enable
    --ena_cabac_5V     : out std_logic;
    --ena_cabac_3_3V   : out std_logic;
    --ena_cabac_VEE    : out std_logic;
    --ena_cabac_B5V    : out std_logic;
    --ena_cabac_B_3_3V : out std_logic;

-- DREB serial number
    reb_sn_onewire : inout std_logic

    );

end WREB_v2_top;

architecture Behavioral of WREB_v2_top is

  component LsstSci is
    generic (
      -------------------------------------------------------------------------
      -- RAFT_DATA_CONVERSION specifies the method used to put the 18 bit
      -- raft data into the 16 bit PGP pipe.
      --
      -- Currently, the choices are:
      -- ZERO_EXTEND_32: Zero extend the data to 32 bits (halves bandwidth)
      -- TRUNC_LOW_2   : Truncate the low 2 bits (loses precision)
      -------------------------------------------------------------------------
      RAFT_DATA_CONVERSION : string
      );      
    port (

      -------------------------------------------------------------------------
      -- FPGA Interface
      -------------------------------------------------------------------------
      FpgaRstL : in std_logic;

      PgpClkP : in  std_logic;
      PgpClkM : in  std_logic;
      PgpRxP  : in  std_logic;
      PgpRxM  : in  std_logic;
      PgpTxP  : out std_logic;
      PgpTxM  : out std_logic;

      -------------------------------------------------------------------------
      -- Clock/Reset Generator Interface
      -------------------------------------------------------------------------
      ClkOut : out std_logic;
      RstOut : out std_logic;

      -------------------------------------------------------------------------
      -- Debug Interface
      -------------------------------------------------------------------------
      PgpLocLinkReadyOut : out std_logic;
      PgpRemLinkReadyOut : out std_logic;

      -------------------------------------------------------------------------
      -- SCI Register Encoder/Decoder Interface
      -------------------------------------------------------------------------
      RegClk    : in  std_logic;
      RegRst    : in  std_logic;
      RegAddr   : out std_logic_vector(23 downto 0);
      RegReq    : out std_logic;
      RegOp     : out std_logic;
      RegDataWr : out std_logic_vector(31 downto 0);
      RegWrEn   : out std_logic_vector(31 downto 0);
      RegAck    : in  std_logic;
      RegFail   : in  std_logic;
      RegDataRd : in  std_logic_vector(31 downto 0);

      -------------------------------------------------------------------------
      -- Data Encoder Interface
      -------------------------------------------------------------------------
      DataClk  : in std_logic;
      DataWrEn : in std_logic;
      DataSOT  : in std_logic;
      DataEOT  : in std_logic;
      DataIn   : in std_logic_vector(17 downto 0);

      -------------------------------------------------------------------------
      -- Status Block Interface
      -------------------------------------------------------------------------
      StatusClk  : in  std_logic;
      StatusRst  : in  std_logic;
      StatusAddr : in  std_logic_vector(23 downto 0);
      StatusReg  : out std_logic_vector(31 downto 0);

      -------------------------------------------------------------------------
      -- Chipscope Control Bus
      -------------------------------------------------------------------------
      CScopeControl : inout std_logic_vector(35 downto 0)
      );

  end component;

  component wreb_v2_cmd_interpeter
    port (
      reset : in std_logic;
      clk   : in std_logic;

-- signals from/to SCI
      regReq           : in  std_logic;  -- with this line the master start a read/write procedure (1 to start)
      regOp            : in  std_logic;  -- this line define if the procedure is read or write (1 to write)
      regAddr          : in  std_logic_vector(23 downto 0);  -- address bus
      statusReg        : in  std_logic_vector(31 downto 0);  -- status reg bus. The RCI handle this bus and this machine pass it to the sure if he wants to read it
      regWrEn          : in  std_logic_vector(31 downto 0);  -- write enable bus. This bus enables the data write bits
      regDataWr_masked : in  std_logic_vector(31 downto 0);  -- data write bus masked. Is the logical AND of data write bus and write enable bus
      regAck           : out std_logic;  -- acknowledge line to activate when the read/write procedure is completed
      regFail          : out std_logic;  -- line to activate when an error occurs during the read/write procedure
      regDataRd        : out std_logic_vector(31 downto 0);  -- data bus to RCI used to transfer read data
      StatusReset      : out std_logic;  -- status block reset

-- Base Register Set signals            
      busy_bus               : in std_logic_vector(31 downto 0);  -- busy bus is composed by the different register sets busy
      time_base_actual_value : in std_logic_vector(63 downto 0);  -- time base value 
      trig_tm_value_SB       : in std_logic_vector(63 downto 0);  -- Status Block trigger time 
      trig_tm_value_TB       : in std_logic_vector(63 downto 0);  -- Time Base trigger time
      trig_tm_value_seq      : in std_logic_vector(63 downto 0);  -- Sequencer Trigger time
      trig_tm_value_V_I      : in std_logic_vector(63 downto 0);  -- Voltage and current sens trigger time
      trig_tm_value_pcb_t    : in std_logic_vector(63 downto 0);  -- PCB temperature Trigger time
--          trig_tm_value_f_adc    : in std_logic_vector(63 downto 0);  -- fast ADC Trigger time

      trigger_ce_bus     : out std_logic_vector(31 downto 0);  -- bus to enable register sets trigger. To trigger a register set that stops itself use en AND val                                      
      trigger_val_bus    : out std_logic_vector(31 downto 0);  -- bus of register sets trigger values  
      load_time_base_lsw : out std_logic;  -- ce signal to load the time base lsw
      load_time_base_MSW : out std_logic;  -- ce signal to load the time base MSW
      cnt_preset         : out std_logic_vector(63 downto 0);  -- preset value for the time base counter

      Mgt_avcc_ok   : in std_logic;
      Mgt_accpll_ok : in std_logic;
      Mgt_avtt_ok   : in std_logic;
      V3_3v_ok      : in std_logic;
      Switch_addr   : in std_logic_vector(7 downto 0);

-- Image parameters
      image_size        : in  std_logic_vector(31 downto 0);  -- this register contains the image size
      image_patter_read : in  std_logic;  -- this register gives the state of image patter gen. 1 is ON
      ccd_sel_read      : in  std_logic_vector(2 downto 0);  -- this register contains the CCD to drive
      image_size_en     : out std_logic;  -- this line enables the register where the image size is written
      image_patter_en   : out std_logic;  -- this register enable the image patter gen. 1 is ON
      ccd_sel_en        : out std_logic;  -- register enable for CCD acquisition selector


-- Sequencer
      seq_time_mem_readbk      : in  std_logic_vector(15 downto 0);  -- time memory read bus
      seq_out_mem_readbk       : in  std_logic_vector(31 downto 0);  -- time memory read bus
      seq_prog_mem_readbk      : in  std_logic_vector(31 downto 0);  -- sequencer program memory read
      seq_time_mem_w_en        : out std_logic;  -- this signal enables the time memory write
      seq_out_mem_w_en         : out std_logic;  -- this signal enables the output memory write
      seq_prog_mem_w_en        : out std_logic;  -- this signal enables the program memory write
      seq_step                 : out std_logic;  -- this signal send the STEP to the sequencer. Valid on in infinite loop (the machine jump out from IL to next function)   
      seq_stop                 : out std_logic;  -- this signal send the STOP to the sequencer. Valid on in infinite loop (the machine jump out from IL to next function)
      enable_conv_shift_in     : in  std_logic;  -- this signal enable the adc_conv shifter (the adc_conv is shifted 1 clk every time is activated)
      enable_conv_shift        : out std_logic;  -- this signal enable the adc_conv shifter (the adc_conv is shifted 1 clk every time is activated)
      init_conv_shift          : out std_logic;  -- this signal initialize the adc_conv shifter (the adc_conv is shifted 1 clk every time is activated)
      start_add_prog_mem_en    : out std_logic;
      start_add_prog_mem_rbk   : in  std_logic_vector(9 downto 0);
      seq_ind_func_mem_we      : out std_logic;
      seq_ind_func_mem_rdbk    : in  std_logic_vector(3 downto 0);
      seq_ind_rep_mem_we       : out std_logic;
      seq_ind_rep_mem_rdbk     : in  std_logic_vector(23 downto 0);
      seq_ind_sub_add_mem_we   : out std_logic;
      seq_ind_sub_add_mem_rdbk : in  std_logic_vector(9 downto 0);
      seq_ind_sub_rep_mem_we   : out std_logic;
      seq_ind_sub_rep_mem_rdbk : in  std_logic_vector(15 downto 0);
      seq_op_code_error        : in  std_logic;
      seq_op_code_error_add    : in  std_logic_vector(9 downto 0);
      seq_op_code_error_reset  : out std_logic;

-- ASPIC

      aspic_config_r_ccd_1 : in  std_logic_vector(15 downto 0);
      aspic_config_r_ccd_2 : in  std_logic_vector(15 downto 0);
      aspic_config_r_ccd_3 : in  std_logic_vector(15 downto 0);
      aspic_op_end         : in  std_logic;
      aspic_start_trans    : out std_logic;
      aspic_start_reset    : out std_logic;
      aspic_nap_mode_en    : out std_logic;
      aspic_nap_mode_in    : in  std_logic;


-- CCD clock rails DAC          
      clk_rail_load_start : out std_logic;
      clk_rail_ldac_start : out std_logic;

-- csgate DAC           
      csgate_load_start : out std_logic;
      csgate_ldac_start : out std_logic;

-- BIAS DAC (former CABAC bias DAC)               
      c_bias_load_start : out std_logic;
      c_bias_ldac_start : out std_logic;

-- DREB voltage and current sensors
      error_V_HTR_voltage   : in std_logic;
      V_HTR_voltage         : in std_logic_vector(15 downto 0);
      error_V_HTR_current   : in std_logic;
      V_HTR_current         : in std_logic_vector(15 downto 0);
      error_V_DREB_voltage  : in std_logic;
      V_DREB_voltage        : in std_logic_vector(15 downto 0);
      error_V_DREB_current  : in std_logic;
      V_DREB_current        : in std_logic_vector(15 downto 0);
      error_V_CLK_H_voltage : in std_logic;
      V_CLK_H_voltage       : in std_logic_vector(15 downto 0);
      error_V_CLK_H_current : in std_logic;
      V_CLK_H_current       : in std_logic_vector(15 downto 0);
      error_V_DPHI_voltage  : in std_logic;
      V_DPHI_voltage        : in std_logic_vector(15 downto 0);
      error_V_DPHI_current  : in std_logic;
      V_DPHI_current        : in std_logic_vector(15 downto 0);
      error_V_ANA_voltage   : in std_logic;
      V_ANA_voltage         : in std_logic_vector(15 downto 0);
      error_V_ANA_current   : in std_logic;
      V_ANA_current         : in std_logic_vector(15 downto 0);
      error_V_OD_voltage    : in std_logic;
      V_OD_voltage          : in std_logic_vector(15 downto 0);
      error_V_OD_current    : in std_logic;
      V_OD_current          : in std_logic_vector(15 downto 0);

-- DREB temperature
      T1_dreb       : in std_logic_vector(15 downto 0);
      T1_dreb_error : in std_logic;
      T2_dreb       : in std_logic_vector(15 downto 0);
      T2_dreb_error : in std_logic;

-- REB temperature gr1
      T1_reb_gr1       : in std_logic_vector(15 downto 0);
      T1_reb_gr1_error : in std_logic;
      T2_reb_gr1       : in std_logic_vector(15 downto 0);
      T2_reb_gr1_error : in std_logic;
      T3_reb_gr1       : in std_logic_vector(15 downto 0);
      T3_reb_gr1_error : in std_logic;
      T4_reb_gr1       : in std_logic_vector(15 downto 0);
      T4_reb_gr1_error : in std_logic;

-- REB temperature gr2
      T1_reb_gr2       : in std_logic_vector(15 downto 0);
      T1_reb_gr2_error : in std_logic;
      T2_reb_gr2       : in std_logic_vector(15 downto 0);
      T2_reb_gr2_error : in std_logic;
      T3_reb_gr2       : in std_logic_vector(15 downto 0);
      T3_reb_gr2_error : in std_logic;
      T4_reb_gr2       : in std_logic_vector(15 downto 0);
      T4_reb_gr2_error : in std_logic;

-- REB temperature gr3
      T1_reb_gr3       : in std_logic_vector(15 downto 0);
      T1_reb_gr3_error : in std_logic;

-- chips temp ADC and mux
      chips_t          : in  std_logic_vector(23 downto 0);
      chips_t_error    : in  std_logic;
      chips_t_busy     : in  std_logic;
      chips_t_start_r  : out std_logic;
      temp_mux_conf_r  : in  std_logic_vector(1 downto 0);
      temp_mux_conf_en : out std_logic;

-- CCD temperature
      ccd_temp_busy  : in  std_logic;
      ccd_temp       : in  std_logic_vector(23 downto 0);
      ccd_temp_start : out std_logic;
      ccd_temp_start_reset     : out std_logic;

-- Bias slow ADC
      bs_adc_busy        : in  std_logic;
      bs_adc_conv_res    : in  array816;
      bs_adc_start_read  : out std_logic;
      bs_adc_start_write : out std_logic;

-- clk rails slow ADC
      ck_adc_busy        : in  std_logic;
      ck_adc_conv_res    : in  array816;
      ck_adc_start_read  : out std_logic;
      ck_adc_start_write : out std_logic;


-- Fast ADC
      --f_adc_num_data_rbk : in  std_logic_vector(23 downto 0);
      --f_adc_out_reg      : in  array1724;
      --f_adc_num_data_en  : out std_logic;

-- DREB 1wire serial number
      dreb_onewire_reset : out std_logic;
      dreb_sn_crc_ok     : in  std_logic;
      dreb_sn_dev_error  : in  std_logic;
      dreb_sn            : in  std_logic_vector(47 downto 0);
      dreb_sn_timeout    : in  std_logic;

-- REB 1wire serial number
      reb_onewire_reset : out std_logic;
      reb_sn_crc_ok     : in  std_logic;
      reb_sn_dev_error  : in  std_logic;
      reb_sn            : in  std_logic_vector(47 downto 0);
      reb_sn_timeout    : in  std_logic;

-- CCD clock enable
      ccd_clk_en_in : in  std_logic;
      ccd_clk_en    : out std_logic;

-- ASPIC reference enable
      aspic_ref_en_in : in  std_logic;
      aspic_ref_en    : out std_logic;
-- ASPIC 5V enable
      aspic_5v_en_in  : in  std_logic;
      aspic_5v_en     : out std_logic;

-- CABAC regulators enable
      CABAC_reg_in : in  std_logic_vector(4 downto 0);
      CABAC_reg_en : out std_logic;

-- back bias switch
      back_bias_sw_rb : in  std_logic;
      back_bias_cl_rb : in  std_logic;
      en_back_bias_sw : out std_logic

      );    
  end component;

  component base_reg_set_top is
    port (
      clk                : in  std_logic;
      reset              : in  std_logic;
      en_time_base_cnt   : in  std_logic;
      load_time_base_lsw : in  std_logic;
      load_time_base_MSW : in  std_logic;
      StatusReset        : in  std_logic;
      trigger_TB         : in  std_logic;
      trigger_seq        : in  std_logic;
      trigger_V_I_read   : in  std_logic;
      trigger_temp_pcb   : in  std_logic;
      trigger_fast_adc   : in  std_logic;
      cnt_preset         : in  std_logic_vector(63 downto 0);
      cnt_busy           : out std_logic;
      cnt_actual_value   : out std_logic_vector(63 downto 0);
      trig_tm_value_SB   : out std_logic_vector(63 downto 0);
      trig_tm_value_TB   : out std_logic_vector(63 downto 0);
      trig_tm_value_seq  : out std_logic_vector(63 downto 0);
      trig_tm_value_V_I  : out std_logic_vector(63 downto 0);
      trig_tm_value_pcb  : out std_logic_vector(63 downto 0);
      trig_tm_value_adc  : out std_logic_vector(63 downto 0)
      );
  end component;

  component sequencer_v3_top is
    port (
      reset                    : in  std_logic;  -- syncronus reset
      clk                      : in  std_logic;  -- clock
      start_sequence           : in  std_logic;
      program_mem_we           : in  std_logic;
      seq_mem_w_add            : in  std_logic_vector(9 downto 0);
      seq_mem_data_in          : in  std_logic_vector(31 downto 0);
      prog_mem_redbk           : out std_logic_vector(31 downto 0);
      program_mem_init_en      : in  std_logic;
      program_mem_init_add_rbk : out std_logic_vector(9 downto 0);
      ind_func_mem_we          : in  std_logic;
      ind_func_mem_redbk       : out std_logic_vector(3 downto 0);
      ind_rep_mem_we           : in  std_logic;
      ind_rep_mem_redbk        : out std_logic_vector(23 downto 0);
      ind_sub_add_mem_we       : in  std_logic;
      ind_sub_add_mem_redbk    : out std_logic_vector(9 downto 0);
      ind_sub_rep_mem_we       : in  std_logic;
      ind_sub_rep_mem_redbk    : out std_logic_vector(15 downto 0);
      time_mem_w_en            : in  std_logic;
      time_mem_readbk          : out std_logic_vector(15 downto 0);
      out_mem_w_en             : in  std_logic;
      out_mem_readbk           : out std_logic_vector(31 downto 0);
      stop_sequence            : in  std_logic;
      step_sequence            : in  std_logic;
      op_code_error_reset      : in  std_logic;
      op_code_error            : out std_logic;
      op_code_error_add        : out std_logic_vector(9 downto 0);
      sequencer_busy           : out std_logic;
      sequencer_out            : out std_logic_vector(31 downto 0);
      end_sequence             : out std_logic
--       CScopeControl                          : inout std_logic_vector(35 downto 0)
      );
  end component;

  component sequencer_aligner_shifter_top is
    generic(start_adc_bit : natural := 12);
    port (
      clk           : in  std_logic;
      reset         : in  std_logic;
      shift_on_en   : in  std_logic;
      shift_on      : in  std_logic;
      init_shift    : in  std_logic;
      sequencer_in  : in  std_logic_vector(31 downto 0);
      shift_on_out  : out std_logic;
      sequencer_out : out std_logic_vector(31 downto 0)
      );
  end component;

  component ADC_data_handler_v4 is
    port (
      reset             : in  std_logic;
      clk               : in  std_logic;
      testmode_rst      : in  std_logic;
      testmode_col      : in  std_logic;
      start_of_img      : in  std_logic;  -- this signal is generated by the user (using the sequencer) and has to arrive before the first trigger
      end_of_img        : in  std_logic;  -- this signal is generated by the user (using the sequencer) and has to arrive after the last  ADC trasfer
      end_sequence      : in  std_logic;  -- this signal is the end of sequence generated by the sequencer and is used as a timeot to generate EOF.
      trigger           : in  std_logic;  -- this signal start the operations (ADC conv and send data to PGP)
      en_test_mode      : in  std_logic;  -- register enable for pattern test mode
      test_mode_in      : in  std_logic;  -- test mode in 
      en_load_ccd_sel   : in  std_logic;  -- register enable for CCD enable
      ccd_sel_in        : in  std_logic_vector(2 downto 0);  -- register to select which CCD acquire (1, 2 or 3) 
      ccd_sel_out       : out std_logic_vector(2 downto 0);  -- register to select which CCD acquire (1, 2 or 3) 
      SOT               : out std_logic;  -- Start of Image
      EOT               : out std_logic;  -- End of Image
      write_enable      : out std_logic;  -- signal to write the image in the PGP
      test_mode_enb_out : out std_logic;
      data_out          : out std_logic_vector(17 downto 0);  -- 18 bits ADC word 
      adc_data_ccd_1    : in  std_logic_vector(15 downto 0);  -- CCD ADC data 
      adc_cnv_ccd_1     : out std_logic;  -- ADC conv
      adc_sck_ccd_1     : out std_logic;  -- ADC serial clock
      adc_data_ccd_2    : in  std_logic_vector(15 downto 0);  -- CCD ADC data 
      adc_cnv_ccd_2     : out std_logic;  -- ADC conv
      adc_sck_ccd_2     : out std_logic;  -- ADC serial clock
      adc_data_ccd_3    : in  std_logic_vector(15 downto 0);  -- CCD ADC data 
      adc_cnv_ccd_3     : out std_logic;  -- ADC conv
      adc_sck_ccd_3     : out std_logic   -- ADC serial clock
      );
  end component;

--component CABAC_1_spi_link_top_mux is
--port (
--              clk                                             : in  std_logic;
--              reset                                           : in  std_logic;
--              start_link_trans                : in  std_logic;
--              start_reset                             : in  std_logic;
--              miso_ccd1                               : in  std_logic;
--              miso_ccd2                               : in  std_logic;
--              miso_ccd3                               : in  std_logic;
--              word2send                               : in  std_logic_vector(31 downto 0);
--              cabac_mosi                              : out std_logic;
--              ss_t_ccd1                               : out std_logic;
--              ss_t_ccd2                               : out std_logic;
--              ss_t_ccd3                               : out std_logic;
--              ss_b_ccd1                               : out std_logic;
--              ss_b_ccd2                               : out std_logic;
--              ss_b_ccd3                               : out std_logic;
--              cabac_sclk                              : out std_logic;
--              cabac_n_reset                   : out std_logic;
--              busy                                            : out std_logic;
--              d_slave_ready_ccd1      : out std_logic;
--              d_slave_ready_ccd2      : out std_logic;
--              d_slave_ready_ccd3      : out std_logic;
--              d_from_slave_ccd1               : out std_logic_vector(15 downto 0);
--              d_from_slave_ccd2               : out std_logic_vector(15 downto 0);
--              d_from_slave_ccd3               : out std_logic_vector(15 downto 0)
--              );
--end component;


  component aspic_3_spi_link_top_mux is
    port (
      clk                : in  std_logic;
      reset              : in  std_logic;
      start_link_trans   : in  std_logic;
      start_reset        : in  std_logic;
      miso_ccd1          : in  std_logic;
      miso_ccd2          : in  std_logic;
      miso_ccd3          : in  std_logic;
      word2send          : in  std_logic_vector(31 downto 0);
      aspic_mosi         : out std_logic;
      ss_t_ccd1          : out std_logic;
      ss_t_ccd2          : out std_logic;
      ss_t_ccd3          : out std_logic;
      ss_b_ccd1          : out std_logic;
      ss_b_ccd2          : out std_logic;
      ss_b_ccd3          : out std_logic;
      aspic_sclk         : out std_logic;
      aspic_n_reset      : out std_logic;
      busy               : out std_logic;
      d_slave_ready_ccd1 : out std_logic;
      d_slave_ready_ccd2 : out std_logic;
      d_slave_ready_ccd3 : out std_logic;
      d_from_slave_ccd1  : out std_logic_vector(15 downto 0);
      d_from_slave_ccd2  : out std_logic_vector(15 downto 0);
      d_from_slave_ccd3  : out std_logic_vector(15 downto 0)
      );
  end component;

  component ad53xx_DAC_top is
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      start_write : in  std_logic;
      start_ldac  : in  std_logic;
      d_to_slave  : in  std_logic_vector(15 downto 0);
      mosi        : out std_logic;
      ss          : out std_logic;
      sclk        : out std_logic;
      ldac        : out std_logic
      );
  end component;

  component dual_ad53xx_DAC_top is
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      start_write : in  std_logic;
      start_ldac  : in  std_logic;
      d_to_slave  : in  std_logic_vector(16 downto 0);
      mosi        : out std_logic;
      ss_dac_0    : out std_logic;
      ss_dac_1    : out std_logic;
      sclk        : out std_logic;
      ldac        : out std_logic
      );
  end component;

  component ltc2945_multi_read_top_wreb is
    port (
      clk                   : in    std_logic;
      reset                 : in    std_logic;
      start_procedure       : in    std_logic;
      busy                  : out   std_logic;
      error_V_HTR_voltage   : out   std_logic;
      V_HTR_voltage_out     : out   std_logic_vector(15 downto 0);
      error_V_HTR_current   : out   std_logic;
      V_HTR_current_out     : out   std_logic_vector(15 downto 0);
      error_V_DREB_voltage  : out   std_logic;
      V_DREB_voltage_out    : out   std_logic_vector(15 downto 0);
      error_V_DREB_current  : out   std_logic;
      V_DREB_current_out    : out   std_logic_vector(15 downto 0);
      error_V_CLK_H_voltage : out   std_logic;
      V_CLK_H_voltage_out   : out   std_logic_vector(15 downto 0);
      error_V_CLK_H_current : out   std_logic;
      V_CLK_H_current_out   : out   std_logic_vector(15 downto 0);
      error_V_DPHI_voltage  : out   std_logic;
      V_DPHI_voltage_out    : out   std_logic_vector(15 downto 0);
      error_V_DPHI_current  : out   std_logic;
      V_DPHI_current_out    : out   std_logic_vector(15 downto 0);
      error_V_ANA_voltage   : out   std_logic;
      V_ANA_voltage_out     : out   std_logic_vector(15 downto 0);
      error_V_ANA_current   : out   std_logic;
      V_ANA_current_out     : out   std_logic_vector(15 downto 0);
      error_V_OD_voltage    : out   std_logic;
      V_OD_voltage_out      : out   std_logic_vector(15 downto 0);
      error_V_OD_current    : out   std_logic;
      V_OD_current_out      : out   std_logic_vector(15 downto 0);
      sda                   : inout std_logic;  --serial data output of i2c bus
      scl                   : inout std_logic  --serial clock output of i2c bus
      );
  end component;

  component adt7420_temp_multiread_2_top is
    port (
      clk             : in    std_logic;
      reset           : in    std_logic;
      start_procedure : in    std_logic;
      busy            : out   std_logic;
      error_T1        : out   std_logic;
      T1_out          : out   std_logic_vector(15 downto 0);
      error_T2        : out   std_logic;
      T2_out          : out   std_logic_vector(15 downto 0);
      sda             : inout std_logic;  --serial data output of i2c bus
      scl             : inout std_logic   --serial clock output of i2c bus
      );
  end component;

  component adt_7420_and_ltc2489_top is
    port (
      clk                : in    std_logic;
      reset              : in    std_logic;
      start_read_board_t : in    std_logic;
      start_read_chip_t  : in    std_logic;
      read_chip_add      : in    std_logic_vector(1 downto 0);
      busy               : out   std_logic;
      error_board_T1     : out   std_logic;
      board_T1_out       : out   std_logic_vector(15 downto 0);
      error_board_T2     : out   std_logic;
      board_T2_out       : out   std_logic_vector(15 downto 0);
      error_board_T3     : out   std_logic;
      board_T3_out       : out   std_logic_vector(15 downto 0);
      error_board_T4     : out   std_logic;
      board_T4_out       : out   std_logic_vector(15 downto 0);
      error_chip_t       : out   std_logic;
      chip_t             : out   std_logic_vector(23 downto 0);
      sda                : inout std_logic;  --serial data output of i2c bus
      scl                : inout std_logic   --serial clock output of i2c bus
      );
  end component;

  component generic_reg_ce_init is
    generic (width : integer);
    port (
      reset    : in  std_logic;         -- syncronus reset
      clk      : in  std_logic;         -- clock
      ce       : in  std_logic;         -- clock enable
      init     : in  std_logic;  -- signal to reset the reg (active high)
      data_in  : in  std_logic_vector(width downto 0);   -- data in
      data_out : out std_logic_vector(width downto 0));  -- data out
  end component;

  component ad7794_top is
    port (
      clk             : in  std_logic;
      reset           : in  std_logic;
      start           : in  std_logic;
      start_reset     : in  std_logic;
      read_write      : in  std_logic;
      ad7794_dout_rdy : in  std_logic;
      reg_add         : in  std_logic_vector(2 downto 0);
      d_to_slave      : in  std_logic_vector(15 downto 0);
      ad7794_din      : out std_logic;
      ad7794_cs       : out std_logic;
      ad7794_sclk     : out std_logic;
      busy            : out std_logic;
      d_from_slave    : out std_logic_vector(23 downto 0)
      );
  end component;

  component max_11046_top
    port (
      clk             : in  std_logic;
      reset           : in  std_logic;
      start_write     : in  std_logic;
      start_read      : in  std_logic;
      EOC             : in  std_logic;
      data_to_adc     : in  std_logic_vector(3 downto 0);
      data_from_adc   : in  std_logic_vector(15 downto 0);
      link_busy       : out std_logic;
      CS              : out std_logic;
      RD              : out std_logic;
      WR              : out std_logic;
      CONVST          : out std_logic;
      SHDN            : out std_logic;
      write_en        : out std_logic;
      data_to_adc_out : out std_logic_vector(3 downto 0);
      cnv_results     : out array816);
  end component;



  --component ad9628_dual_fast_adc_top is
  --  port (
  --    clk                  : in  std_logic;
  --    reset                : in  std_logic;
  --    start                : in  std_logic;
  --    num_data_to_read_en  : in  std_logic;
  --    num_data_to_read     : in  std_logic_vector(23 downto 0);
  --    adc_data_in_cha      : in  std_logic_vector(11 downto 0);
  --    adc_data_in_chb      : in  std_logic_vector(11 downto 0);
  --    dcoa                 : in  std_logic;
  --    dcob                 : in  std_logic;
  --    adc_clk_en           : out std_logic;
  --    busy                 : out std_logic;
  --    adc_pdown            : out std_logic;
  --    write_en_sci         : out std_logic;
  --    adc_data_SOF         : out std_logic;
  --    adc_data_EOF         : out std_logic;
  --    num_data_to_read_rbk : out std_logic_vector(23 downto 0);
  --    adc_data_out_cha     : out std_logic_vector(11 downto 0);
  --    adc_data_out_chb     : out std_logic_vector(11 downto 0);
  --    out_fast_adc_reg     : out array1724
  --    );
  --end component;

  component onewire_iface
    generic (
      CheckCRC   : boolean;
      ADD_PULLUP : boolean;
      CLK_DIV    : integer range 0 to 12);
    port (
      sys_clk     : in    std_logic;    -- system clock (50Mhz)
      latch_reset : in    std_logic;
      sys_reset   : in    std_logic;    -- active high syn. reset 
      dq          : inout std_logic;    -- connect to the 1-wire bus
      dev_error   : out   std_logic;
      data        : out   std_logic_vector(7 downto 0);    -- data output
      data_valid  : out   std_logic;    -- data output valid (20us strobe)
      crc_ok      : out   std_logic;    -- crc ok signal (active high)
      timeout     : out   std_logic;    -- timeout signal ~10ms
      sn_data     : out   std_logic_vector(47 downto 0));  -- parallel out
  end component;

  --component clk_2MHz_generator is
  --  port (
  --    clk             : in  std_logic;
  --    reset           : in  std_logic;
  --    clk_2MHz_en     : in  std_logic;
  --    clk_2MHz_en_in  : in  std_logic;
  --    clk_2MHz_en_out : out std_logic;
  --    clk_2MHz_out    : out std_logic
  --    );
  --end component;

  component ff_ce is
    port (
      reset    : in  std_logic;         -- syncronus reset
      clk      : in  std_logic;         -- clock
      data_in  : in  std_logic;         -- data in
      ce       : in  std_logic;         -- clock enable
      data_out : out std_logic);        -- data out
  end component;

  component ff_ce_pres is
    port (
      preset   : in  std_logic;
      clk      : in  std_logic;
      data_in  : in  std_logic;
      ce       : in  std_logic;
      data_out : out std_logic
      ); 
  end component;

  component led_blink is
    port (
      clk_in  : in  std_logic;
      led_out : out std_logic);
  end component;

  component dcm_base
    port
      (                                 -- Clock in ports
        CLK_IN1  : in  std_logic;
                                        -- Clock out ports
        CLK_OUT1 : out std_logic;
                                        -- Status and control signals
        RESET    : in  std_logic;
        LOCKED   : out std_logic
        );
  end component;

  component DREB_V2_icon
    port (
      CONTROL0 : inout std_logic_vector(35 downto 0);
      CONTROL1 : inout std_logic_vector(35 downto 0));
  end component;

  component DREB_v2_ila
    port (
      CONTROL : inout std_logic_vector(35 downto 0);
      CLK     : in    std_logic;
      TRIG0   : in    std_logic_vector(95 downto 0));
  end component;

-- Clocks
--      signal pgpRefClk                : std_logic;
  signal clk_100_Mhz : std_logic;
  signal pgp_usr_clk : std_logic;

-- Reset
  signal n_rst      : std_logic;
  signal usrRst     : std_logic;
  signal NusrRst    : std_logic;
  signal sync_res   : std_logic;
  signal sync_res_1 : std_logic;
  signal sync_res_2 : std_logic;

-- SCI signals
  signal pgpLocLinkReady : std_logic;
  signal pgpRemLinkReady : std_logic;
  signal regReq          : std_logic;
  signal regOp           : std_logic;
  signal RegAddr         : std_logic_vector(23 downto 0);
  signal RegDataWr       : std_logic_vector(31 downto 0);
  signal regAck          : std_logic;
  signal regFail         : std_logic;
  signal RegDataRd       : std_logic_vector(31 downto 0);
  signal RegWrEn         : std_logic_vector(31 downto 0);
  signal dataWrEn        : std_logic;
  signal dataSOT         : std_logic;
  signal dataEOT         : std_logic;
  signal image_in        : std_logic_vector(17 downto 0);
  signal StatusAddr      : std_logic_vector(23 downto 0);
  signal StatusReg       : std_logic_vector(31 downto 0);
  signal StatusRst       : std_logic;

-- CMD interpreter signals
  signal regDataWr_masked   : std_logic_vector(31 downto 0);
  signal busy_bus           : std_logic_vector(31 downto 0);
  signal trigger_ce_bus     : std_logic_vector(31 downto 0);
  signal trigger_val_bus    : std_logic_vector(31 downto 0);
  signal load_time_base_lsw : std_logic;
  signal load_time_base_MSW : std_logic;
  signal cnt_preset         : std_logic_vector(63 downto 0);

  -- BRS signals
  signal time_base_actual_value : std_logic_vector(63 downto 0);
  signal trig_tm_value_SB       : std_logic_vector(63 downto 0);
  signal trig_tm_value_TB       : std_logic_vector(63 downto 0);
  signal trig_tm_value_seq      : std_logic_vector(63 downto 0);
  signal trig_tm_value_V_I      : std_logic_vector(63 downto 0);
  signal trig_tm_value_pcb_t    : std_logic_vector(63 downto 0);
--  signal trig_tm_value_fast_adc : std_logic_vector(63 downto 0);
  signal time_base_busy         : std_logic;

  -- sequencer signals
  signal sequencer_busy           : std_logic;
  signal seq_time_mem_readbk      : std_logic_vector(15 downto 0);
  signal seq_out_mem_readbk       : std_logic_vector(31 downto 0);
  signal seq_prog_mem_readbk      : std_logic_vector(31 downto 0);
  signal seq_time_mem_w_en        : std_logic;
  signal seq_out_mem_w_en         : std_logic;
  signal seq_prog_mem_w_en        : std_logic;
  signal seq_start                : std_logic;
  signal seq_step                 : std_logic;
  signal seq_stop                 : std_logic;
  signal sequencer_outputs        : std_logic_vector(31 downto 0);
  signal sequencer_outputs_int    : std_logic_vector(31 downto 0);
  signal enable_conv_shift        : std_logic;
  signal enable_conv_shift_out    : std_logic;
  signal init_conv_shift          : std_logic;
  signal end_sequence             : std_logic;
  signal start_add_prog_mem_en    : std_logic;
  signal start_add_prog_mem_rbk   : std_logic_vector(9 downto 0);
  signal seq_ind_func_mem_we      : std_logic;
  signal seq_ind_func_mem_rdbk    : std_logic_vector(3 downto 0);
  signal seq_ind_rep_mem_we       : std_logic;
  signal seq_ind_rep_mem_rdbk     : std_logic_vector(23 downto 0);
  signal seq_ind_sub_add_mem_we   : std_logic;
  signal seq_ind_sub_add_mem_rdbk : std_logic_vector(9 downto 0);
  signal seq_ind_sub_rep_mem_we   : std_logic;
  signal seq_ind_sub_rep_mem_rdbk : std_logic_vector(15 downto 0);
  signal seq_op_code_error        : std_logic;
  signal seq_op_code_error_reset  : std_logic;
  signal seq_op_code_error_add    : std_logic_vector(9 downto 0);

-- CABAC config signals
  --signal cabac_start_trans    : std_logic;
  --signal cabac_start_reset    : std_logic;
  --signal cabac_busy           : std_logic;
  --signal cabac_config_r_ccd_1 : std_logic_vector (15 downto 0);
  --signal CABAC_mosi_int       : std_logic;
  --signal CABAC_sclk_int       : std_logic;

-- ASPIC config signals
  signal aspic_start_trans    : std_logic;
  signal aspic_start_reset    : std_logic;
  signal aspic_busy           : std_logic;
  signal aspic_config_r_ccd_1 : std_logic_vector (15 downto 0);
--      signal aspic_config_r_ccd_2     : std_logic_vector (15 downto 0);
--      signal aspic_config_r_ccd_3     : std_logic_vector (15 downto 0);
  signal ASPIC_mosi_int       : std_logic;
  signal ASPIC_sclk_int       : std_logic;
  signal ASPIC_miso           : std_logic;
  signal aspic_miso_sel       : std_logic;

  signal aspic_nap_mode_en : std_logic;
  signal aspic_nap_mode    : std_logic;


-- ASPIC CCD 1
  signal ASPIC_r_up_ccd_1   : std_logic;
  signal ASPIC_r_down_ccd_1 : std_logic;
  signal ASPIC_clamp_ccd_1  : std_logic;
  signal ASPIC_reset_ccd_1  : std_logic;


-- CCD 1 signals
  signal par_clk_ccd_1    : std_logic_vector(3 downto 0);
  signal ser_clk_ccd_1    : std_logic_vector(2 downto 0);
  signal reset_gate_ccd_1 : std_logic;
--   signal adc_data_ccd_1                      : std_logic_vector(15 downto 0); 


-- Image handler signals
  signal image_size        : std_logic_vector(31 downto 0);
  signal image_patter_read : std_logic;
  signal image_size_en     : std_logic;
  signal image_patter_en   : std_logic;
  signal ADC_trigger       : std_logic;
--      signal CCD_sel_en                               : std_logic;
  signal CCD_sel           : std_logic_vector(2 downto 0);
  signal start_of_img      : std_logic;
  signal end_of_img        : std_logic;
  signal pattern_reset     : std_logic;

-- CCD clock rails DAC                  
  signal clk_rail_load_start : std_logic;
  signal clk_rail_ldac_start : std_logic;

-- csgate DAC           
  signal csgate_load_start : std_logic;
  signal csgate_ldac_start : std_logic;

-- CABAC bias
  signal c_bias_load_start : std_logic;
  signal c_bias_ldac_start : std_logic;

-- ltc2945 V & I sensors read
  signal V_I_read_start        : std_logic;
  signal V_I_busy              : std_logic;
  signal error_V_HTR_voltage   : std_logic;
  signal V_HTR_voltage         : std_logic_vector(15 downto 0);
  signal error_V_HTR_current   : std_logic;
  signal V_HTR_current         : std_logic_vector(15 downto 0);
  signal error_V_DREB_voltage  : std_logic;
  signal V_DREB_voltage        : std_logic_vector(15 downto 0);
  signal error_V_DREB_current  : std_logic;
  signal V_DREB_current        : std_logic_vector(15 downto 0);
  signal error_V_CLK_H_voltage : std_logic;
  signal V_CLK_H_voltage       : std_logic_vector(15 downto 0);
  signal error_V_CLK_H_current : std_logic;
  signal V_CLK_H_current       : std_logic_vector(15 downto 0);
  signal error_V_DPHI_voltage  : std_logic;
  signal V_DPHI_voltage        : std_logic_vector(15 downto 0);
  signal error_V_DPHI_current  : std_logic;
  signal V_DPHI_current        : std_logic_vector(15 downto 0);
  signal error_V_ANA_voltage   : std_logic;
  signal V_ANA_voltage         : std_logic_vector(15 downto 0);
  signal error_V_ANA_current   : std_logic;
  signal V_ANA_current         : std_logic_vector(15 downto 0);
  signal error_V_OD_voltage    : std_logic;
  signal V_OD_voltage          : std_logic_vector(15 downto 0);
  signal error_V_OD_current    : std_logic;
  signal V_OD_current          : std_logic_vector(15 downto 0);

-- PCB temperature
  signal temp_read_start : std_logic;
  signal temp_busy       : std_logic;

-- DREB temperature
  signal DREB_temp_busy : std_logic;
  signal T1_dreb        : std_logic_vector(15 downto 0);
  signal T1_dreb_error  : std_logic;
  signal T2_dreb        : std_logic_vector(15 downto 0);
  signal T2_dreb_error  : std_logic;

--REB temperature gr1
  signal REB_temp_busy_gr1 : std_logic;
  signal T1_reb_gr1        : std_logic_vector(15 downto 0);
  signal T1_reb_gr1_error  : std_logic;
  signal T2_reb_gr1        : std_logic_vector(15 downto 0);
  signal T2_reb_gr1_error  : std_logic;
  signal T3_reb_gr1        : std_logic_vector(15 downto 0);
  signal T3_reb_gr1_error  : std_logic;
  signal T4_reb_gr1        : std_logic_vector(15 downto 0);
  signal T4_reb_gr1_error  : std_logic;

-- chips temeperature ADC and mux
  signal chips_t          : std_logic_vector(23 downto 0);
  signal chips_t_error    : std_logic;
  signal chips_t_start_r  : std_logic;
  signal temp_mux_conf_r  : std_logic_vector(1 downto 0);
  signal temp_mux_conf_en : std_logic;

-- CCD temperature 
  signal ccd_temp_busy  : std_logic;
  signal ccd_temp       : std_logic_vector(23 downto 0);
  signal ccd_temp_start : std_logic;
   signal ccd_temp_start_reset : std_logic;

-- bias slow adc

  signal bs_adc_busy              : std_logic;
  signal bs_adc_conv_res          : array816;
  signal bs_adc_start_read        : std_logic;
  signal bs_adc_start_write       : std_logic;
  signal bs_adc_write_en          : std_logic;
  signal bs_adc_data_to_adc_out   : std_logic_vector(3 downto 0);
  signal bs_adc_data_from_adc_int : std_logic_vector(15 downto 0);


-- clk rails slow ADC
  signal ck_adc_busy              : std_logic;
  signal ck_adc_conv_res          : array816;
  signal ck_adc_start_read        : std_logic;
  signal ck_adc_start_write       : std_logic;
  signal ck_adc_write_en          : std_logic;
  signal ck_adc_data_to_adc_out   : std_logic_vector(3 downto 0);
  signal ck_adc_data_from_adc_int : std_logic_vector(15 downto 0);


-- fast ADC 
  --signal fast_adc_clk             : std_logic;
  --signal fast_adc_clk_en          : std_logic;
  --signal fast_adc_busy            : std_logic;
  --signal fast_adc_start           : std_logic;
  --signal fast_adc_data_to_read_en : std_logic;
  --signal fast_adc_out_reg         : array1724;
  --signal fast_adc_data_to_r_rdbk  : std_logic_vector(23 downto 0);
  --signal dco_mux0_int             : std_logic;
  --signal dco_mux1_int             : std_logic;

-- REB 1wire serial number
  signal reb_onewire_reset : std_logic;
  signal reb_sn_crc_ok     : std_logic;
  signal reb_sn_dev_error  : std_logic;
  signal reb_sn            : std_logic_vector(47 downto 0);
  signal reb_sn_timeout    : std_logic;

-- CCD clock enable
  signal ccd_clk_en_out_int : std_logic;
  signal ccd_clk_en         : std_logic;

-- ASPIC reference enable
  signal aspic_ref_en_out_int : std_logic;
  signal aspic_ref_en         : std_logic;

  -- ASPIC 5V enable
  signal aspic_5v_en_out_int : std_logic;
  signal aspic_5v_en         : std_logic;

-- CABAC regulators enable
  signal CABAC_reg_in : std_logic_vector(4 downto 0);
  signal CABAC_reg_en : std_logic;

------ MISC ------
-- test led
  signal test_led_int : std_logic_vector(5 downto 0);
  signal dcm_locked   : std_logic;
  signal test_port    : std_logic_vector(3 downto 0);

-- CABAC_pulse
  signal cabac_pulse_ccd_1 : std_logic;

-- back bias switch signals
  signal en_back_bias_sw     : std_logic;
  signal back_bias_sw_int    : std_logic;
  signal back_bias_clamp_int : std_logic;

-- this line enables the output buffers 
  signal enable_io : std_logic;

  signal ASPIC_ss_t_ccd_1_int : std_logic;
  signal ASPIC_ss_b_ccd_1_int : std_logic;
  signal ASPIC_spi_reset_int  : std_logic;


  --signal CABAC_ss_t_ccd_1_int  : std_logic;
  --signal CABAC_ss_b_ccd_1_int  : std_logic;
  --signal CABAC_reset_ccd_1_int : std_logic;

-- chipscope
  signal CONTROL0       : std_logic_vector(35 downto 0);
  signal CONTROL1       : std_logic_vector(35 downto 0);
  signal DREB_v2_ila_in : std_logic_vector(95 downto 0);
--signal chipscope_clk          : std_logic;

  signal LTC2945_SDA_int    : std_logic;
  signal LTC2945_SCL_int    : std_logic;
  signal test_i2c           : std_logic;
  signal reb_sn_onewire_int : std_logic;

  signal adc_cnv_ccd_1_int : std_logic;
  signal adc_sck_ccd_1_int : std_logic;

  signal asic_spi_mosi_int : std_logic;
  signal asic_spi_sclk_int : std_logic;

  signal gpio_0_int : std_logic;
  signal gpio_1_int : std_logic;

  signal bs_adc_CS_int     : std_logic;
  signal bs_adc_RD_int     : std_logic;
  signal bs_adc_WR_int     : std_logic;
  signal bs_adc_CONVST_int : std_logic;
  signal bs_adc_SHDN_int   : std_logic;

  signal ck_adc_CS_int     : std_logic;
  signal ck_adc_RD_int     : std_logic;
  signal ck_adc_WR_int     : std_logic;
  signal ck_adc_CONVST_int : std_logic;
  signal ck_adc_SHDN_int   : std_logic;


begin


  test_led_int(5) <= pgpLocLinkReady;
  test_led_int(3) <= pgpRemLinkReady;
--test_led_int(5)                               <= '1'; 



  regDataWr_masked         <= regDataWr and regWrEn;
  StatusAddr(23 downto 10) <= (others => '0');
  StatusAddr(9 downto 0)   <= regAddr(9 downto 0);

  busy_bus <= x"000000" & "000" & temp_busy & V_I_busy & sequencer_busy & time_base_busy & '0';


-- trigger signals
  seq_start       <= trigger_val_bus(2) and trigger_ce_bus(2);
  V_I_read_start  <= trigger_val_bus(3) and trigger_ce_bus(3);
  temp_read_start <= trigger_val_bus(4) and trigger_ce_bus(4);
--  fast_adc_start  <= trigger_val_bus(5) and trigger_ce_bus(5);

-- temperature signals
  temp_busy <= DREB_temp_busy or REB_temp_busy_gr1;

-- chips temperature mux
  Tmux_ena_ccd_1 <= '1';
  Tmux_a0_ccd_1  <= temp_mux_conf_r(0);
  Tmux_a1_ccd_1  <= temp_mux_conf_r(0);

  aspic_miso_sel <= ASPIC_ss_t_ccd_1_int and (not ASPIC_ss_b_ccd_1_int);
  ASPIC_miso     <= asic_spi_miso_t when aspic_miso_sel = '0' else asic_spi_miso_b;

------------ Chips and video ADC NAP mode lines ------------ 
--      ASPIC_nap_ccd_1         <= '0'; 
  ASPIC_nap_ccd_1 <= aspic_nap_mode;
--  CABAC_ro_ccd_1  <= '1';
  adc_buff_pd     <= '1';


------------ Sequencer's signals assignment ------------
  ADC_trigger <= sequencer_outputs(12);
-- CCD 1
--      ASPIC_r_up_ccd_1                <= sequencer_outputs(0) and CCD_sel(0);
--      ASPIC_r_down_ccd_1      <= sequencer_outputs(1) and CCD_sel(0);

-- this inversion is to correct a n/p inversion on the WREB board

  ASPIC_r_up_ccd_1   <= CCD_sel(0) and (not sequencer_outputs(0));
  ASPIC_r_down_ccd_1 <= CCD_sel(0) and (not sequencer_outputs(1));

  ASPIC_reset_ccd_1 <= sequencer_outputs(2) and CCD_sel(0);
  ASPIC_clamp_ccd_1 <= sequencer_outputs(3) and CCD_sel(0);
  ser_clk_ccd_1(0)  <= sequencer_outputs(4) and CCD_sel(0);
  ser_clk_ccd_1(1)  <= sequencer_outputs(5) and CCD_sel(0);
  ser_clk_ccd_1(2)  <= sequencer_outputs(6) and CCD_sel(0);
  reset_gate_ccd_1  <= sequencer_outputs(7) and CCD_sel(0);
  par_clk_ccd_1(0)  <= sequencer_outputs(8) and CCD_sel(0);
  par_clk_ccd_1(1)  <= sequencer_outputs(9) and CCD_sel(0);
  par_clk_ccd_1(2)  <= sequencer_outputs(10) and CCD_sel(0);
  par_clk_ccd_1(3)  <= sequencer_outputs(11) and CCD_sel(0);

  ADC_trigger       <= sequencer_outputs(12);
  start_of_img      <= sequencer_outputs(13);
  end_of_img        <= sequencer_outputs(14);
  cabac_pulse_ccd_1 <= sequencer_outputs(15);
  pattern_reset     <= sequencer_outputs(16);

--      test_led_int(4)         <= sequencer_outputs(15);
  test_led_int(4) <= sequencer_outputs(16);
  gpio_0_int      <= sequencer_outputs(16);
  gpio_1_int      <= sequencer_outputs(17);

------------ assignment for test ------------
--      test_port(10 downto 0)  <= sequencer_outputs(10 downto 0);
--      test_port(11)                           <= sequencer_outputs(12);
--      test_port(12)                           <= sequencer_outputs(16); 
  test_port(2) <= sequencer_outputs_int(12);


------------ misc ------------
-- this is a leftover form CABAC
  asic_spi_sclk_int <= ASPIC_sclk_int;
  asic_spi_mosi_int <= ASPIC_mosi_int;

  asic_spi_sclk <= asic_spi_sclk_int;
  asic_spi_mosi <= asic_spi_mosi_int;

  enable_io <= '0';                     -- 1 = disable 

--      ASPIC_ss_t_ccd_1                <= '0';
--      ASPIC_ss_b_ccd_1                <= '0';
--      ASPIC_spi_reset         <= '0';
--       
--      CABAC_ss_t_ccd_1                <= '0';
--      CABAC_ss_b_ccd_1                <= '0';
--      CABAC_reset_ccd_1               <= '0'; 

  ASPIC_ss_t_ccd_1 <= ASPIC_ss_t_ccd_1_int;
  ASPIC_ss_b_ccd_1 <= ASPIC_ss_b_ccd_1_int;
  ASPIC_spi_reset  <= ASPIC_spi_reset_int;

  --CABAC_ss_t_ccd_1  <= CABAC_ss_t_ccd_1_int;
  --CABAC_ss_b_ccd_1  <= CABAC_ss_b_ccd_1_int;
  --CABAC_reset_ccd_1 <= CABAC_reset_ccd_1_int;


-- CABAC power supply enable    
--      ena_cabac_5V                            <= '1';
--      ena_cabac_3_3V                          <= '1';
--      ena_cabac_VEE                           <= '1';
--      ena_cabac_B5V                           <= '1';
--      ena_cabac_B_3_3V                        <= '1';

  --ena_cabac_5V     <= CABAC_reg_in(0);
  --ena_cabac_3_3V   <= CABAC_reg_in(1);
  --ena_cabac_VEE    <= CABAC_reg_in(2);
  --ena_cabac_B5V    <= CABAC_reg_in(3);
  --ena_cabac_B_3_3V <= CABAC_reg_in(4);

  PWR_SYNC <= '0';  -- do not put 1 on this line on WREB 1. The DC/DC cannot be driven at 3.3V

  LTC2945_SDA    <= LTC2945_SDA_int;
  LTC2945_SCL    <= LTC2945_SCl_int;
--  LTC2945_SCl_int <= '1';
  reb_sn_onewire <= reb_sn_onewire_int;

  adc_cnv_ccd_1 <= adc_cnv_ccd_1_int;
  adc_sck_ccd_1 <= adc_sck_ccd_1_int;

  bs_adc_CS     <= bs_adc_CS_int;
  bs_adc_RD     <= bs_adc_RD_int;
  bs_adc_WR     <= bs_adc_WR_int;
  bs_adc_CONVST <= bs_adc_CONVST_int;
  bs_adc_SHDN   <= bs_adc_SHDN_int;

  ck_adc_CS     <= ck_adc_CS_int;
  ck_adc_RD     <= ck_adc_RD_int;
  ck_adc_WR     <= ck_adc_WR_int;
  ck_adc_CONVST <= ck_adc_CONVST_int;
  ck_adc_SHDN   <= ck_adc_SHDN_int;

  LsstSci_0 : LsstSci
    generic map (
      -------------------------------------------------------------------------
      -- RAFT_DATA_CONVERSION specifies the method used to put the 18 bit
      -- raft data into the 16 bit PGP pipe.
      --
      -- Currently, the choices are:
      -- ZERO_EXTEND_32: Zero extend the data to 32 bits (halves bandwidth)
      -- TRUNC_LOW_2   : Truncate the low 2 bits (loses precision)
      -------------------------------------------------------------------------
      RAFT_DATA_CONVERSION => "ZERO_EXTEND_32"
      )      
    port map (
      -------------------------------------------------------------------------
      -- FPGA Interface
      -------------------------------------------------------------------------

      FpgaRstL => n_rst,

      PgpClkP            => PgpRefClk_P,
      PgpClkM            => PgpRefClk_M,
      PgpRxP             => PgpRx_p,
      PgpRxM             => PgpRx_m,
      PgpTxP             => PgpTx_p,
      PgpTxM             => PgpTx_m,
      -------------------------------------------------------------------------
      -- Clock/Reset Generator Interface
      -------------------------------------------------------------------------
      ClkOut             => pgp_usr_clk,
      RstOut             => usrRst,
      -------------------------------------------------------------------------
      -- Debug Interface
      -------------------------------------------------------------------------
      PgpLocLinkReadyOut => pgpLocLinkReady,
      PgpRemLinkReadyOut => pgpRemLinkReady,
      -------------------------------------------------------------------------
      -- SCI Register Encoder/Decoder Interface
      -------------------------------------------------------------------------
      RegClk             => clk_100_Mhz,
      RegRst             => sync_res,
      RegAddr            => RegAddr,
      RegReq             => regReq,
      RegOp              => regOp,
      RegDataWr          => RegDataWr,
      RegWrEn            => RegWrEn,
      RegAck             => regAck,
      RegFail            => regFail,
      RegDataRd          => RegDataRd,
      -------------------------------------------------------------------------
      -- Data Encoder Interface
      -------------------------------------------------------------------------
      DataClk            => clk_100_MHz,
      DataWrEn           => dataWrEn,
      DataSOT            => dataSOT,
      DataEOT            => dataEOT,
      DataIn             => image_in,
      -------------------------------------------------------------------------
      -- Status Block Interface
      -------------------------------------------------------------------------
      StatusClk          => clk_100_Mhz,
      StatusRst          => StatusRst,
      StatusAddr         => StatusAddr,
      StatusReg          => StatusReg,
      -------------------------------------------------------------------------
      -- Chipscope Control Bus
      -------------------------------------------------------------------------
      CScopeControl      => CONTROL1
--      CScopeControl => open
      );


  wreb_v2_cmd_interpeter_0 : wreb_v2_cmd_interpeter
    port map (
      reset                    => sync_res,
      clk                      => clk_100_MHz,
-- signals from/to SCI
      regReq                   => regReq,  -- with this line the master start a read/write procedure (1 to start)
      regOp                    => regOp,  -- this line define if the procedure is read or write (1 to write)
      regAddr                  => RegAddr,  -- address bus
      statusReg                => StatusReg,  -- status reg bus. The RCI handle this bus and this machine pass it to the sure if he wants to read it
      regWrEn                  => RegWrEn,  -- write enable bus. This bus enables the data write bits
      regDataWr_masked         => regDataWr_masked,  -- data write bus masked. Is the logical AND of data write bus and write enable bus
      regAck                   => regAck,  -- acknowledge line to activate when the read/write procedure is completed
      regFail                  => regFail,  -- line to activate when an error occurs during the read/write procedure
      regDataRd                => RegDataRd,  -- data bus to RCI used to transfer read data
      StatusReset              => StatusRst,  -- status block reset
-- Base Register Set signals            
      busy_bus                 => busy_bus,  -- busy bus is composed by the different register sets busy
      time_base_actual_value   => time_base_actual_value,  -- time base value 
      trig_tm_value_SB         => trig_tm_value_SB,  -- Status Block trigger time 
      trig_tm_value_TB         => trig_tm_value_TB,  -- Time Base trigger time
      trig_tm_value_seq        => trig_tm_value_seq,  -- Sequencer Trigger time
      trig_tm_value_V_I        => trig_tm_value_V_I,  -- Voltage and current sens trigger time
      trig_tm_value_pcb_t      => trig_tm_value_pcb_t,  -- PCB temperature Trigger time
--      trig_tm_value_f_adc      => trig_tm_value_fast_adc,  -- fast ADC Trigger time
      trigger_ce_bus           => trigger_ce_bus,  -- bus to enable register sets trigger. To trigger a register set that stops itself use en AND val                                      
      trigger_val_bus          => trigger_val_bus,  -- bus of register sets trigger values  
      load_time_base_lsw       => load_time_base_lsw,  -- ce signal to load the time base lsw
      load_time_base_MSW       => load_time_base_MSW,  -- ce signal to load the time base MSW
      cnt_preset               => cnt_preset,  -- preset value for the time base counter
      Mgt_avcc_ok              => '0',
      Mgt_accpll_ok            => '0',
      Mgt_avtt_ok              => '0',
      V3_3v_ok                 => '0',
      --     Switch_addr              => r_add,
      Switch_addr              => '0' & r_add,
-- Image parameters
      image_size               => x"00000000",  -- this register contains the image size (no longer used)
      image_patter_read        => image_patter_read,  -- this register gives the state of image patter gen. 1 is ON
      ccd_sel_read             => CCD_sel,  -- this register contains the CCD to drive
      image_size_en            => open,  -- this line enables the register where the image size is written
      image_patter_en          => image_patter_en,  -- this register enable the image patter gen. 1 is ON
      ccd_sel_en               => open,  -- on WREB only first stripe is active                                                  -- register enable for CCD acquisition selector 
-- Sequencer
      seq_time_mem_readbk      => seq_time_mem_readbk,  -- time memory read bus
      seq_out_mem_readbk       => seq_out_mem_readbk,  -- time memory read bus
      seq_prog_mem_readbk      => seq_prog_mem_readbk,  -- sequencer program memory read
      seq_time_mem_w_en        => seq_time_mem_w_en,  -- this signal enables the time memory write
      seq_out_mem_w_en         => seq_out_mem_w_en,  -- this signal enables the output memory write
      seq_prog_mem_w_en        => seq_prog_mem_w_en,  -- this signal enables the program memory write
      seq_step                 => seq_step,  -- this signal send the STEP to the sequencer. Valid on in infinite loop (the machine jump out from IL to next function)   
      seq_stop                 => seq_stop,  -- this signal send the STOP to the sequencer. Valid on in infinite loop (the machine jump out from IL to next function)
      enable_conv_shift_in     => enable_conv_shift_out,  -- this signal enable the adc_conv shifter (the adc_conv is shifted 1 clk every time is activated)
      enable_conv_shift        => enable_conv_shift,  -- this signal enable the adc_conv shifter (the adc_conv is shifted 1 clk every time is activated)
      init_conv_shift          => init_conv_shift,  -- this signal initialize the adc_conv shifter (the adc_conv is shifted 1 clk every time is activated)
      start_add_prog_mem_en    => start_add_prog_mem_en,
      start_add_prog_mem_rbk   => start_add_prog_mem_rbk,
      seq_ind_func_mem_we      => seq_ind_func_mem_we,
      seq_ind_func_mem_rdbk    => seq_ind_func_mem_rdbk,
      seq_ind_rep_mem_we       => seq_ind_rep_mem_we,
      seq_ind_rep_mem_rdbk     => seq_ind_rep_mem_rdbk,
      seq_ind_sub_add_mem_we   => seq_ind_sub_add_mem_we,
      seq_ind_sub_add_mem_rdbk => seq_ind_sub_add_mem_rdbk,
      seq_ind_sub_rep_mem_we   => seq_ind_sub_rep_mem_we,
      seq_ind_sub_rep_mem_rdbk => seq_ind_sub_rep_mem_rdbk,
      seq_op_code_error        => seq_op_code_error,
      seq_op_code_error_add    => seq_op_code_error_add,
      seq_op_code_error_reset  => seq_op_code_error_reset,

-- CABAC
      --cabac_config_r_ccd_1 => cabac_config_r_ccd_1,
      --cabac_op_end         => cabac_busy,
      --cabac_start_trans    => cabac_start_trans,
      --cabac_start_reset    => cabac_start_reset,
-- ASPIC
      aspic_config_r_ccd_1 => aspic_config_r_ccd_1,
      aspic_config_r_ccd_2 => x"0000",
      aspic_config_r_ccd_3 => x"0000",
      aspic_op_end         => aspic_busy,
      aspic_start_trans    => aspic_start_trans,
      aspic_start_reset    => aspic_start_reset,
      aspic_nap_mode_en    => aspic_nap_mode_en,
      aspic_nap_mode_in    => aspic_nap_mode,

-- CCD clock rails DAC          
      clk_rail_load_start   => clk_rail_load_start,
      clk_rail_ldac_start   => clk_rail_ldac_start,
-- csgate DAC           
      csgate_load_start     => csgate_load_start,
      csgate_ldac_start     => csgate_ldac_start,
--  BIAS DAC (former CABAC bias DAC)                
      c_bias_load_start     => c_bias_load_start,
      c_bias_ldac_start     => c_bias_ldac_start,
-- DREB voltage and current sensors
      error_V_HTR_voltage   => error_V_HTR_voltage,
      V_HTR_voltage         => V_HTR_voltage,
      error_V_HTR_current   => error_V_HTR_current,
      V_HTR_current         => V_HTR_current,
      error_V_DREB_voltage  => error_V_DREB_voltage,
      V_DREB_voltage        => V_DREB_voltage,
      error_V_DREB_current  => error_V_DREB_current,
      V_DREB_current        => V_DREB_current,
      error_V_CLK_H_voltage => error_V_CLK_H_voltage,
      V_CLK_H_voltage       => V_CLK_H_voltage,
      error_V_CLK_H_current => error_V_CLK_H_current,
      V_CLK_H_current       => V_CLK_H_current,
      error_V_DPHI_voltage  => error_V_DPHI_voltage,
      V_DPHI_voltage        => V_DPHI_voltage,
      error_V_DPHI_current  => error_V_DPHI_current,
      V_DPHI_current        => V_DPHI_current,
      error_V_ANA_voltage   => error_V_ANA_voltage,
      V_ANA_voltage         => V_ANA_voltage,
      error_V_ANA_current   => error_V_ANA_current,
      V_ANA_current         => V_ANA_current,
      error_V_OD_voltage    => error_V_OD_voltage,
      V_OD_voltage          => V_OD_voltage,
      error_V_OD_current    => error_V_OD_current,
      V_OD_current          => V_OD_current,
-- DREB temperature
      T1_dreb               => T1_dreb,
      T1_dreb_error         => T1_dreb_error,
      T2_dreb               => T2_dreb,
      T2_dreb_error         => T2_dreb_error,
-- REB temperature gr1
      T1_reb_gr1            => T1_reb_gr1,
      T1_reb_gr1_error      => T1_reb_gr1_error,
      T2_reb_gr1            => T2_reb_gr1,
      T2_reb_gr1_error      => T2_reb_gr1_error,
      T3_reb_gr1            => T3_reb_gr1,
      T3_reb_gr1_error      => T3_reb_gr1_error,
      T4_reb_gr1            => T4_reb_gr1,
      T4_reb_gr1_error      => T4_reb_gr1_error,
-- REB temperature gr2
      T1_reb_gr2            => x"0000",
      T1_reb_gr2_error      => '0',
      T2_reb_gr2            => x"0000",
      T2_reb_gr2_error      => '0',
      T3_reb_gr2            => x"0000",
      T3_reb_gr2_error      => '0',
      T4_reb_gr2            => x"0000",
      T4_reb_gr2_error      => '0',
-- REB temperature gr3
      T1_reb_gr3            => x"0000",
      T1_reb_gr3_error      => '0',
-- chips temp ADC and mux
      chips_t               => chips_t,
      chips_t_error         => chips_t_error,
      chips_t_busy          => REB_temp_busy_gr1,
      chips_t_start_r       => chips_t_start_r,
      temp_mux_conf_r       => temp_mux_conf_r,
      temp_mux_conf_en      => temp_mux_conf_en,
-- CCD temperature
      ccd_temp_busy         => ccd_temp_busy,
      ccd_temp              => ccd_temp,
      ccd_temp_start        => ccd_temp_start,
       ccd_temp_start_reset => ccd_temp_start_reset,

-- Bias slow ADC
      bs_adc_busy        => bs_adc_busy,
      bs_adc_conv_res    => bs_adc_conv_res,
      bs_adc_start_read  => bs_adc_start_read,
      bs_adc_start_write => bs_adc_start_write,

-- clk rails slow ADC
      ck_adc_busy        => ck_adc_busy,
      ck_adc_conv_res    => ck_adc_conv_res,
      ck_adc_start_read  => ck_adc_start_read,
      ck_adc_start_write => ck_adc_start_write,

-- Fast ADC
      --f_adc_num_data_rbk    => fast_adc_data_to_r_rdbk,
      --f_adc_out_reg         => fast_adc_out_reg,
      --f_adc_num_data_en     => fast_adc_data_to_read_en,
-- DREB 1wire serial number
      dreb_onewire_reset => open,
      dreb_sn_crc_ok     => '0',
      dreb_sn_dev_error  => '0',
      dreb_sn            => x"000000000000",
      dreb_sn_timeout    => '0',
-- REB 1wire serial number
      reb_onewire_reset  => reb_onewire_reset,
      reb_sn_crc_ok      => reb_sn_crc_ok,
      reb_sn_dev_error   => reb_sn_dev_error,
      reb_sn             => reb_sn,
      reb_sn_timeout     => reb_sn_timeout,
-- CCD clock enable
      ccd_clk_en_in      => ccd_clk_en_out_int,
      ccd_clk_en         => ccd_clk_en,
-- ASPIC reference enable
      aspic_ref_en_in    => aspic_ref_en_out_int,
      aspic_ref_en       => aspic_ref_en,
-- ASPIC 5V enable
      aspic_5v_en_in     => aspic_5v_en_out_int,
      aspic_5v_en        => aspic_5v_en,
-- CABAC regulators enable 
      CABAC_reg_in       => CABAC_reg_in,
      CABAC_reg_en       => CABAC_reg_en,
-- back bias switch
      back_bias_sw_rb    => back_bias_sw_int,
      back_bias_cl_rb    => back_bias_clamp_int,
      en_back_bias_sw    => en_back_bias_sw
      );

  base_reg_set : base_reg_set_top
    port map (
      clk                => clk_100_MHz,
      reset              => sync_res,
      en_time_base_cnt   => trigger_ce_bus(1),
      load_time_base_lsw => load_time_base_lsw,
      load_time_base_MSW => load_time_base_MSW,
      StatusReset        => StatusRst,
      trigger_TB         => trigger_val_bus(1),
      trigger_seq        => seq_start,
      trigger_V_I_read   => V_I_read_start,
      trigger_temp_pcb   => temp_read_start,
      trigger_fast_adc   => '0',
      cnt_preset         => cnt_preset,
      cnt_busy           => time_base_busy,
      cnt_actual_value   => time_base_actual_value,
      trig_tm_value_SB   => trig_tm_value_SB,
      trig_tm_value_TB   => trig_tm_value_TB,
      trig_tm_value_seq  => trig_tm_value_seq,
      trig_tm_value_V_I  => trig_tm_value_V_I,
      trig_tm_value_pcb  => trig_tm_value_pcb_t,
      trig_tm_value_adc  => open
      );

  sequencer_v3_0 : sequencer_v3_top
    port map (
      reset                    => sync_res,
      clk                      => clk_100_MHz,
      start_sequence           => seq_start,
      program_mem_we           => seq_prog_mem_w_en,
      seq_mem_w_add            => regAddr(9 downto 0),
      seq_mem_data_in          => regDataWr_masked,
      prog_mem_redbk           => seq_prog_mem_readbk,
      program_mem_init_en      => start_add_prog_mem_en,
      program_mem_init_add_rbk => start_add_prog_mem_rbk,
      ind_func_mem_we          => seq_ind_func_mem_we,
      ind_func_mem_redbk       => seq_ind_func_mem_rdbk,
      ind_rep_mem_we           => seq_ind_rep_mem_we,
      ind_rep_mem_redbk        => seq_ind_rep_mem_rdbk,
      ind_sub_add_mem_we       => seq_ind_sub_add_mem_we,
      ind_sub_add_mem_redbk    => seq_ind_sub_add_mem_rdbk,
      ind_sub_rep_mem_we       => seq_ind_sub_rep_mem_we,
      ind_sub_rep_mem_redbk    => seq_ind_sub_rep_mem_rdbk,
      time_mem_w_en            => seq_time_mem_w_en,
      time_mem_readbk          => seq_time_mem_readbk,
      out_mem_w_en             => seq_out_mem_w_en,
      out_mem_readbk           => seq_out_mem_readbk,
      stop_sequence            => seq_stop,
      step_sequence            => seq_step,
      op_code_error_reset      => seq_op_code_error_reset,
      op_code_error            => seq_op_code_error,
      op_code_error_add        => seq_op_code_error_add,
      sequencer_busy           => sequencer_busy,
      sequencer_out            => sequencer_outputs_int,
      end_sequence             => end_sequence
--       CScopeControl => CONTROL1
      );

  sequencer_aligner_shifter : sequencer_aligner_shifter_top
    generic map(start_adc_bit => 12)
    port map (
      clk           => clk_100_Mhz,
      reset         => sync_res,
      shift_on_en   => enable_conv_shift,
      shift_on      => regDataWr_masked(0),
      init_shift    => init_conv_shift,
      sequencer_in  => sequencer_outputs_int,
      shift_on_out  => enable_conv_shift_out,
      sequencer_out => sequencer_outputs
      );

  Image_data_handler_0 : ADC_data_handler_v4
    port map (
      reset             => sync_res,
      clk               => clk_100_Mhz,
      testmode_rst      => pattern_reset,
      testmode_col      => sequencer_outputs(8),
      start_of_img      => start_of_img,  -- this signal is generated by the user (using the sequencer) and has to arrive before the first trigger 
      end_of_img        => end_of_img,  -- this signal is generated by the user (using the sequencer) and has to arrive after the last  ADC trasfer 
      end_sequence      => end_sequence,  -- this signal is the end of sequence generated by the sequencer and is used as a timeot to generate EOF.
      trigger           => ADC_trigger,  -- this signal start the operations (ADC conv and send data to PGP)
      en_test_mode      => image_patter_en,  -- register enable for pattern test mode
      test_mode_in      => regDataWr_masked(0),  -- test mode in 
      en_load_ccd_sel   => '1',  -- for WREB only first stripe is active                                                         -- register enable for CCD enable
      ccd_sel_in        => "001",  -- for WREB only first stripe is active                                                         -- register to select which CCD acquire (1, 2 or 3) 
      ccd_sel_out       => CCD_sel,  -- register to select which CCD acquire (1, 2 or 3) 
      SOT               => dataSOT,     -- Start of Image
      EOT               => dataEOT,     -- End of Image
      write_enable      => dataWrEn,    -- signal to write the image in the PGP
      test_mode_enb_out => image_patter_read,
      data_out          => image_in,    -- 18 bits ADC word 
      adc_data_ccd_1    => adc_data_ccd_1,   -- CCD ADC data 
      adc_cnv_ccd_1     => adc_cnv_ccd_1_int,    -- ADC conv
      adc_sck_ccd_1     => adc_sck_ccd_1_int,    -- ADC serial clock
      adc_data_ccd_2    => x"0000",  -- for WREB only first stripe is active                                 -- CCD ADC data 
      adc_cnv_ccd_2     => open,  -- for WREB only first stripe is active                                 -- ADC conv
      adc_sck_ccd_2     => open,  -- for WREB only first stripe is active                                 -- ADC serial clock
      adc_data_ccd_3    => x"0000",  -- for WREB only first stripe is active                                 -- CCD ADC data 
      adc_cnv_ccd_3     => open,  -- for WREB only first stripe is active                         -- ADC conv
      adc_sck_ccd_3     => open  -- for WREB only first stripe is active                         -- ADC serial clock
      );


  --CABAC_1_spi_link_top_mux_0 : CABAC_1_spi_link_top_mux
  --  port map (
  --    clk                => clk_100_Mhz,
  --    reset              => sync_res,
  --    start_link_trans   => cabac_start_trans,
  --    start_reset        => cabac_start_reset,
  --    miso_ccd1          => asic_spi_miso,
  --    miso_ccd2          => '0',
  --    miso_ccd3          => '0',
  --    word2send          => regDataWr_masked,
  --    cabac_mosi         => CABAC_mosi_int,
  --    ss_t_ccd1          => CABAC_ss_t_ccd_1_int,
  --    ss_t_ccd2          => open,
  --    ss_t_ccd3          => open,
  --    ss_b_ccd1          => CABAC_ss_b_ccd_1_int,
  --    ss_b_ccd2          => open,
  --    ss_b_ccd3          => open,
  --    cabac_sclk         => CABAC_sclk_int,
  --    cabac_n_reset      => CABAC_reset_ccd_1_int,
  --    busy               => cabac_busy,
  --    d_slave_ready_ccd1 => open,
  --    d_slave_ready_ccd2 => open,
  --    d_slave_ready_ccd3 => open,
  --    d_from_slave_ccd1  => cabac_config_r_ccd_1,
  --    d_from_slave_ccd2  => open,
  --    d_from_slave_ccd3  => open
  --    );

  aspic_3_spi_link_top_mux_0 : aspic_3_spi_link_top_mux
    port map (
      clk                => clk_100_Mhz,
      reset              => sync_res,
      start_link_trans   => aspic_start_trans,
      start_reset        => aspic_start_reset,
      miso_ccd1          => ASPIC_miso,
      miso_ccd2          => '0',
      miso_ccd3          => '0',
      word2send          => regDataWr_masked,
      aspic_mosi         => ASPIC_mosi_int,
      ss_t_ccd1          => ASPIC_ss_t_ccd_1_int,
      ss_t_ccd2          => open,
      ss_t_ccd3          => open,
      ss_b_ccd1          => ASPIC_ss_b_ccd_1_int,
      ss_b_ccd2          => open,
      ss_b_ccd3          => open,
      aspic_sclk         => ASPIC_sclk_int,
      aspic_n_reset      => ASPIC_spi_reset_int,
      busy               => aspic_busy,
      d_slave_ready_ccd1 => open,
      d_slave_ready_ccd2 => open,
      d_slave_ready_ccd3 => open,
      d_from_slave_ccd1  => aspic_config_r_ccd_1,
      d_from_slave_ccd2  => open,
      d_from_slave_ccd3  => open
      );

  aspic_nap_mode_ff : ff_ce
    port map (
      reset    => sync_res,
      clk      => clk_100_Mhz,
      data_in  => regDataWr_masked(0),
      ce       => aspic_nap_mode_en,
      data_out => aspic_nap_mode);  

  csgate_prog : ad53xx_DAC_top
    port map (
      clk         => clk_100_Mhz,
      reset       => sync_res,
      start_write => csgate_load_start,
      start_ldac  => csgate_ldac_start,
      d_to_slave  => regDataWr_masked(15 downto 0),
      mosi        => din_CSGATE,
      ss          => sync_CSGATE,
      sclk        => sclk_CSGATE,
      ldac        => ldac_CSGATE
      );

  clk_rails_prog : dual_ad53xx_DAC_top
    port map (
      clk         => clk_100_Mhz,
      reset       => sync_res,
      start_write => clk_rail_load_start,
      start_ldac  => clk_rail_ldac_start,
      d_to_slave  => regDataWr_masked(16 downto 0),
      mosi        => din_RAILS,
      ss_dac_0    => sync_RAILS_dac0,
      ss_dac_1    => sync_RAILS_dac1,
      sclk        => sclk_RAILS,
      ldac        => ldac_RAILS
      );

  c_bias_prog : ad53xx_DAC_top
    port map (
      clk         => clk_100_Mhz,
      reset       => sync_res,
      start_write => c_bias_load_start,
      start_ldac  => c_bias_ldac_start,
      d_to_slave  => regDataWr_masked(15 downto 0),
      mosi        => din_C_BIAS,
      ss          => sync_C_BIAS,
      sclk        => sclk_C_BIAS,
      ldac        => ldac_C_BIAS
      );

  ltc2945_V_I_sens : ltc2945_multi_read_top_wreb
    port map (
      clk                   => clk_100_Mhz,
      reset                 => sync_res,
      start_procedure       => V_I_read_start,
      busy                  => V_I_busy,
      error_V_HTR_voltage   => error_V_HTR_voltage,
      V_HTR_voltage_out     => V_HTR_voltage,
      error_V_HTR_current   => error_V_HTR_current,
      V_HTR_current_out     => V_HTR_current,
      error_V_DREB_voltage  => error_V_DREB_voltage,
      V_DREB_voltage_out    => V_DREB_voltage,
      error_V_DREB_current  => error_V_DREB_current,
      V_DREB_current_out    => V_DREB_current,
      error_V_CLK_H_voltage => error_V_CLK_H_voltage,
      V_CLK_H_voltage_out   => V_CLK_H_voltage,
      error_V_CLK_H_current => error_V_CLK_H_current,
      V_CLK_H_current_out   => V_CLK_H_current,
      error_V_DPHI_voltage  => error_V_DPHI_voltage,
      V_DPHI_voltage_out    => V_DPHI_voltage,
      error_V_DPHI_current  => error_V_DPHI_current,
      V_DPHI_current_out    => V_DPHI_current,
      error_V_ANA_voltage   => error_V_ANA_voltage,
      V_ANA_voltage_out     => V_ANA_voltage,
      error_V_ANA_current   => error_V_ANA_current,
      V_ANA_current_out     => V_ANA_current,
      error_V_OD_voltage    => error_V_OD_voltage,
      V_OD_voltage_out      => V_OD_voltage,
      error_V_OD_current    => error_V_OD_current,
      V_OD_current_out      => V_OD_current,
      sda                   => LTC2945_SDA_int,  --serial data output of i2c bus 
      scl                   => LTC2945_SCl_int  --serial clock output of i2c bus
      );

  DREB_temp_read : adt7420_temp_multiread_2_top
    port map (
      clk             => clk_100_Mhz,
      reset           => sync_res,
      start_procedure => temp_read_start,
      busy            => DREB_temp_busy,
      error_T1        => T1_dreb_error,
      T1_out          => T1_dreb,
      error_T2        => T2_dreb_error,
      T2_out          => T2_dreb,
      sda             => DREB_temp_sda,  --serial data output of i2c bus 
      scl             => DREB_temp_scl   --serial clock output of i2c bus
      );

  board_and_chip_temp : adt_7420_and_ltc2489_top
    port map (
      clk                => clk_100_Mhz,
      reset              => sync_res,
      start_read_board_t => temp_read_start,
      start_read_chip_t  => chips_t_start_r,
      read_chip_add      => regDataWr_masked(1 downto 0),
      busy               => REB_temp_busy_gr1,
      error_board_T1     => T1_reb_gr1_error,
      board_T1_out       => T1_reb_gr1,
      error_board_T2     => T2_reb_gr1_error,
      board_T2_out       => T2_reb_gr1,
      error_board_T3     => T3_reb_gr1_error,
      board_T3_out       => T3_reb_gr1,
      error_board_T4     => T4_reb_gr1_error,
      board_T4_out       => T4_reb_gr1,
      error_chip_t       => chips_t_error,
      chip_t             => chips_t,
      sda                => Temp_adc_sda_ccd_1,  --serial data output of i2c bus 
      scl                => Temp_adc_scl_ccd_1  --serial clock output of i2c bus
      );

  chips_t_mux_reg : generic_reg_ce_init
    generic map(width => 1)
    port map (
      reset    => sync_res,
      clk      => clk_100_Mhz,
      ce       => temp_mux_conf_en,
      init     => '0',
      data_in  => regDataWr_masked(1 downto 0),
      data_out => temp_mux_conf_r
      );

  ccd_temperature_sensor : ad7794_top
    port map (
      clk             => clk_100_Mhz,
      reset           => sync_res,
      start           => ccd_temp_start,
      start_reset     => ccd_temp_start_reset,
      read_write      => regDataWr_masked(19),
      ad7794_dout_rdy => dout_24ADC,
      reg_add         => regDataWr_masked(18 downto 16),
      d_to_slave      => regDataWr_masked(15 downto 0),
      ad7794_din      => din_24ADC,
      ad7794_cs       => csb_24ADC,
      ad7794_sclk     => sclk_24ADC,
      busy            => ccd_temp_busy,
      d_from_slave    => ccd_temp
      );



  bs_adc : max_11046_top
    port map (
      clk             => clk_100_Mhz,
      reset           => sync_res,
      start_write     => bs_adc_start_write,
      start_read      => bs_adc_start_read,
      EOC             => bs_adc_EOC,
      data_to_adc     => regDataWr_masked(3 downto 0),
      data_from_adc   => bs_adc_data_from_adc_int,
      link_busy       => bs_adc_busy,
      CS              => bs_adc_CS_int,
      RD              => bs_adc_RD_int,
      WR              => bs_adc_WR_int,
      CONVST          => bs_adc_CONVST_int,
      SHDN            => open,
--      SHDN            => bs_adc_SHDN_int,
      write_en        => bs_adc_write_en,
      data_to_adc_out => bs_adc_data_to_adc_out,
      cnv_results     => bs_adc_conv_res
      );
-- shoutdown seams not working. After shutdown the ADC gives wrong values. 
  bs_adc_SHDN_int <= '0';

  ck_adc : max_11046_top
    port map (
      clk             => clk_100_Mhz,
      reset           => sync_res,
      start_write     => ck_adc_start_write,
      start_read      => ck_adc_start_read,
      EOC             => ck_adc_EOC,
      data_to_adc     => regDataWr_masked(3 downto 0),
      data_from_adc   => ck_adc_data_from_adc_int,
      link_busy       => ck_adc_busy,
      CS              => ck_adc_CS_int,
      RD              => ck_adc_RD_int,
      WR              => ck_adc_WR_int,
      CONVST          => ck_adc_CONVST_int,
      SHDN            => open,
--      SHDN            => ck_adc_SHDN,
      write_en        => ck_adc_write_en,
      data_to_adc_out => ck_adc_data_to_adc_out,
      cnv_results     => ck_adc_conv_res
      );

  ck_adc_SHDN <= '0';




  --ad9628_dual_fast_adc_top_0 : ad9628_dual_fast_adc_top
  --  port map (
  --    clk                  => clk_100_Mhz,
  --    reset                => sync_res,
  --    start                => fast_adc_start,
  --    num_data_to_read_en  => fast_adc_data_to_read_en,
  --    num_data_to_read     => regDataWr_masked(23 downto 0),
  --    adc_data_in_cha      => d_mux0,
  --    adc_data_in_chb      => d_mux1,
  --    dcoa                 => dco_mux0_int,
  --    dcob                 => dco_mux1_int,
  --    adc_clk_en           => fast_adc_clk_en,
  --    busy                 => fast_adc_busy,
  --    adc_pdown            => pd_adc_mux,
  --    write_en_sci         => open,
  --    adc_data_SOF         => open,
  --    adc_data_EOF         => open,
  --    num_data_to_read_rbk => fast_adc_data_to_r_rdbk,
  --    adc_data_out_cha     => open,
  --    adc_data_out_chb     => open,
  --    out_fast_adc_reg     => fast_adc_out_reg
  --    );

--  dcdc_clk_gen : clk_2MHz_generator
--    port map (
--      clk             => clk_100_Mhz,
--      reset           => sync_res,
--      clk_2MHz_en     => dcdc_clk_en,
--      clk_2MHz_en_in  => regDataWr_masked(0),
--      clk_2MHz_en_out => dcdc_clk_en_out,
----              clk_2MHz_out    => PWR_SYNC do not activate this line on WREB 1 the DC/DC cannot be driven at 3.3V
--      clk_2MHz_out    => open
--      ); 

--led_blink : clock_divider  
--      generic map (clock_in_frequency => 100000000, 
--                              clock_out_frequency => 2)
--      port map ( 
--              clk_in  => clk_100_Mhz,
--              clk_out  => test_led_int(3) 
--              );

  ccd_clk_enable_ff : ff_ce
    port map (
      reset    => sync_res,
      clk      => clk_100_Mhz,
      data_in  => regDataWr_masked(0),
      ce       => ccd_clk_en,
      data_out => ccd_clk_en_out_int);

  ASPIC_ref_enable_ff : ff_ce_pres
    port map (
      preset   => sync_res,
      clk      => clk_100_Mhz,
      data_in  => regDataWr_masked(0),
      ce       => aspic_ref_en,
      data_out => aspic_ref_en_out_int);

  ASPIC_ref_sd <= aspic_ref_en_out_int;

  ASPIC_5v_enable_ff : ff_ce_pres
    port map (
      preset   => sync_res,
      clk      => clk_100_Mhz,
      data_in  => regDataWr_masked(0),
      ce       => aspic_5v_en,
      data_out => aspic_5v_en_out_int);

  ASPIC_5V_sd <= aspic_5v_en_out_int;

  led_blink_0 : led_blink
    port map (
      clk_in  => clk_100_Mhz,
      led_out => test_led_int(2));


  REB_1wire_sn : onewire_iface
    generic map (
      CheckCRC   => true,
      ADD_PULLUP => false,
      CLK_DIV    => 12)
    port map(
      sys_clk     => clk_100_Mhz,
      latch_reset => sync_res,
      sys_reset   => reb_onewire_reset,
      crc_ok      => reb_sn_crc_ok,
      dev_error   => reb_sn_dev_error,
      data        => open,
      data_valid  => open,
      sn_data     => reb_sn,
      timeout     => reb_sn_timeout,
      dq          => reb_sn_onewire_int
      );

  back_bias_sw : ff_ce
    port map (
      reset    => sync_res,
      clk      => clk_100_Mhz,
      data_in  => regDataWr_masked(0),
      ce       => en_back_bias_sw,
      data_out => back_bias_sw_int); 

  back_bias_clamp_int <= not back_bias_sw_int;

  back_bias_reg : ff_ce
    port map (
      reset    => sync_res,
      clk      => clk_100_Mhz,
      data_in  => back_bias_sw_int,
      ce       => '1',
      data_out => backbias_ssbe); 

  back_bias_clamp_reg : ff_ce_pres
    port map (
      preset   => sync_res,
      clk      => clk_100_Mhz,
      data_in  => back_bias_clamp_int,
      ce       => '1',
      data_out => backbias_clamp);       

  --CABAC_regulators_reg : generic_reg_ce_init
  --  generic map(width => 4)
  --  port map (
  --    reset    => sync_res,
  --    clk      => clk_100_Mhz,
  --    ce       => CABAC_reg_en,
  --    init     => '0',
  --    data_in  => regDataWr_masked(4 downto 0),
  --    data_out => CABAC_reg_in
  --    );

-- clock 
  
  dcm_base_0 : dcm_base
    port map
    (                                   -- Clock in ports 
      CLK_IN1  => pgp_usr_clk,
      -- Clock out ports 
      CLK_OUT1 => clk_100_Mhz,
      -- Status and control signals 
      RESET    => '0',
      LOCKED   => test_led_int(1)); 

---- check master clk
--
--  ODDR_inst : ODDR generic map(DDR_CLK_EDGE => "OPPOSITE_EDGE",
--                               INIT         => '0',
--                               SRTYPE       => "SYNC") 
--    port map (
--      Q  => test_port(0),
--      C  => pgp_usr_clk,
--      CE => '1',
--      D1 => '1',
--      D2 => '0',
--      R  => '0',
--      S  => '0'
--      );
--
--sys_clk_in_buffer: IBUFGDS
--generic map (
--              DIFF_TERM               => TRUE,
--              IBUF_LOW_PWR    => FALSE,
--              IOSTANDARD              => "DEFAULT")
--port map (
--              I       => sysclk_p,
--              IB      => sysclk_m,
--              O       => clk_100_Mhz);

-- fast adc clock inputs

  --fast_adc_clk_0 : IBUFG
  --  port map (
  --    I => dco_mux0,
  --    O => dco_mux0_int);

  --fast_adc_clk_1 : IBUFG
  --  port map (
  --    I => dco_mux1,
  --    O => dco_mux1_int);

-- Resets
  -- Power on reset (goes to PGP part)
  Ureset : IBUF port map (O => n_rst, I => Pwron_Rst_L);

  NusrRst <= not usrRst;

                                        -- sync reset for the user part (from PGP)
  flop1_res : FD port map (D => usrRst, C => clk_100_Mhz, Q => sync_res_1);
  flop2_res : FD port map (D => sync_res_1, C => clk_100_Mhz, Q => sync_res_2);
  flop3_res : FD port map (D => sync_res_2, C => clk_100_Mhz, Q => sync_res);

-- Clock conditioning
  -- PGP serdes clk
--      U_PgpRefClk : IBUFDS port map (I  => PgpRefClk_P,
--                                  IB => PgpRefClk_M,
--                                  O  => pgpRefClk);

------ MISC ------                                  
-- leds
  Utest_led0 : OBUF port map (O => TEST_LED(0), I => test_led_int(0));
  Utest_led1 : OBUF port map (O => TEST_LED(1), I => test_led_int(1));
  Utest_led2 : OBUF port map (O => TEST_LED(2), I => test_led_int(2));
  Utest_led3 : OBUF port map (O => TEST_LED(3), I => test_led_int(3));
  Utest_led4 : OBUF port map (O => TEST_LED(4), I => test_led_int(4));
  Utest_led5 : OBUF port map (O => TEST_LED(5), I => test_led_int(5));



-- CCD 1
  U_ASPIC_r_up_ccd_1 : OBUFTDS port map (I  => ASPIC_r_up_ccd_1,
                                         T  => enable_io,
                                         O  => ASPIC_r_up_ccd_1_p,
                                         OB => ASPIC_r_up_ccd_1_n);

  U_ASPIC_r_down_ccd_1 : OBUFTDS port map (I  => ASPIC_r_down_ccd_1,
                                           T  => enable_io,
                                           O  => ASPIC_r_down_ccd_1_p,
                                           OB => ASPIC_r_down_ccd_1_n);

  U_ASPIC_clamp_ccd_1 : OBUFTDS port map (I  => ASPIC_clamp_ccd_1,
                                          T  => enable_io,
                                          O  => ASPIC_clamp_ccd_1_p,
                                          OB => ASPIC_clamp_ccd_1_n);

  U_ASPIC_reset_ccd_1 : OBUFTDS port map (I  => ASPIC_reset_ccd_1,
                                          T  => enable_io,
                                          O  => ASPIC_reset_ccd_1_p,
                                          OB => ASPIC_reset_ccd_1_n);

  par_clk_ccd_1_generate :
  for I in 0 to 3 generate
    U_par_clk_ccd_1 : OBUFTDS
      port map (I  => par_clk_ccd_1(I),
                T  => enable_io,
                O  => par_clk_ccd_1_p(I),
                OB => par_clk_ccd_1_n(I));
  end generate;

  ser_clk_ccd_1_generate :
  for I in 0 to 2 generate
    U_ser_clk_ccd_1 : OBUFTDS
      port map (I  => ser_clk_ccd_1(I),
                T  => enable_io,
                O  => ser_clk_ccd_1_p(I),
                OB => ser_clk_ccd_1_n(I));
  end generate;

  U_reset_gate_ccd_1 : OBUFTDS port map (I  => reset_gate_ccd_1,
                                         T  => enable_io,
                                         O  => reset_gate_ccd_1_p,
                                         OB => reset_gate_ccd_1_n);

  U_pulse_t_ccd_1 : OBUFTDS port map (I  => cabac_pulse_ccd_1,
                                      T  => enable_io,
                                      O  => pulse_ccd_1_p,
                                      OB => pulse_ccd_1_n);



-- bias slow adc tri state buffer

  bs_adc_iobuf_generate :
  for i in 0 to 3 generate
    bs_IOBF : IOBUF
      port map (
        O  => bs_adc_data_from_adc_int(i),
        IO => bs_adc_data_from_adc_dcr(i),
        I  => bs_adc_data_to_adc_out(i),
        T  => bs_adc_write_en
        );
  end generate;


  --bs_adc_iobuf_generate :
  --for i in 0 to 3 generate
  --  bs_IOBF : IOBUF
  --    port map (
  --      O  => bs_adc_data_to_adc_out(i),
  --      IO => bs_adc_data_from_adc_dcr(i),
  --      I  => bs_adc_data_from_adc_int(i),
  --      T  => bs_adc_write_en
  --      );
  --end generate;

  ck_adc_iobuf_generate :
  for i in 0 to 3 generate
    ck_IOBF : IOBUF
      port map (
        O  => ck_adc_data_from_adc_int(i),
        IO => ck_adc_data_from_adc_dcr(i),
        I  => ck_adc_data_to_adc_out(i),
        T  => ck_adc_write_en
        );
  end generate;




  --ck_adc_iobuf_generate :
  --for i in 0 to 3 generate
  --  ck_IOBF : IOBUF
  --    port map (
  --      O  => ck_adc_data_to_adc_out(i),
  --      IO => ck_adc_data_from_adc_dcr(i),
  --      I  => ck_adc_data_from_adc_int(i),
  --      T  => ck_adc_write_en
  --      );
  --end generate;

  bs_adc_data_from_adc_int(15 downto 4) <= bs_adc_data_from_adc;
  ck_adc_data_from_adc_int(15 downto 4) <= ck_adc_data_from_adc;


  CCD_clk_en_buffer : OBUFDS
    port map (I  => ccd_clk_en_out_int,
              O  => ccd_clk_en_out_p,
              OB => ccd_clk_en_out_n);

  gpio_0_buffer : OBUFDS
    port map (I  => gpio_0_int,
              O  => gpio_0_p,
              OB => gpio_0_n);

  gpio_1_buffer : OBUFDS
    port map (I  => gpio_1_int,
              O  => gpio_1_p,
              OB => gpio_1_n);

  gpio_0_dir <= '0';                    -- must be 0 to work as receiver
  gpio_1_dir <= '0';                    -- must be 0 to work as receiver

  --OBUFDS_inst : OBUFTDS
  --  generic map (
  --    IOSTANDARD => "DEFAULT",
  --    SLEW       => "low")
  --  port map (
  --    O  => clk_mux_p,
  --    OB => clk_mux_n,
  --    T  => enable_io,
  --    I  => fast_adc_clk
  --    );

-- test points
-- test port
  Utest0 : OBUF port map (O => TEST(0), I => test_port(0));
  Utest1 : OBUF port map (O => TEST(1), I => test_port(1));
  Utest2 : OBUF port map (O => TEST(2), I => test_port(2));
  Utest3 : OBUF port map (O => TEST(3), I => test_port(3));
  --Utest4  : OBUF port map (O => TEST(4), I => test_port(4));
  --Utest5  : OBUF port map (O => TEST(5), I => test_port(5));
  --Utest6  : OBUF port map (O => TEST(6), I => test_port(6));
  --Utest7  : OBUF port map (O => TEST(7), I => test_port(7));
  --Utest8  : OBUF port map (O => TEST(8), I => test_port(8));
  --Utest9  : OBUF port map (O => TEST(9), I => test_port(9));
  --Utest10 : OBUF port map (O => TEST(10), I => test_port(10));
  --Utest11 : OBUF port map (O => TEST(11), I => test_port(11));
  --Utest12 : OBUF port map (O => TEST(12), I => test_port(12));


-- chipscope

  DREB_v2_0 : DREB_V2_icon
    port map (
      CONTROL0 => CONTROL0,
      CONTROL1 => CONTROL1);

  DREB_v2_ila_0 : DREB_v2_ila
    port map (
      CONTROL => CONTROL0,
      CLK     => clk_100_Mhz,
      TRIG0   => DREB_v2_ila_in);

  
  DREB_v2_ila_in(0) <= regReq;
  DREB_v2_ila_in(1) <= regOp;


--DREB_v2_ila_in(49 downto 2)                   <= reb_sn;

  DREB_v2_ila_in(25 downto 2)  <= regAddr;
-- 
--  DREB_v2_ila_in(57 downto 26) <= regDataWr;
--  DREB_v2_ila_in(89 downto 58) <= regDataRd;

--DREB_v2_ila_in(90)                                            <= seq_start;
--DREB_v2_ila_in(91)                                            <= test_i2c;
--DREB_v2_ila_in(92)                                            <= LTC2945_SDA_int;
--DREB_v2_ila_in(93)                                            <= V_I_busy; 
--DREB_v2_ila_in(94)                                            <= dataSOT;
--DREB_v2_ila_in(95)                                            <= dataEOT;
--DREB_v2_ila_in(90)                                            <= LTC2945_SCL_int;

--DREB_v2_ila_in(90)                                            <= adc_cnv_ccd_1_int;
--DREB_v2_ila_in(91)                                            <= adc_sck_ccd_1_int;
--DREB_v2_ila_in(92)                                            <= ADC_trigger;
--DREB_v2_ila_in(93)                                            <= reb_sn_dev_error; 
--DREB_v2_ila_in(94)                                            <= dataSOT;
--DREB_v2_ila_in(95)                                            <= dataEOT;


--DREB_v2_ila_in(88) <= asic_spi_sclk_int;
--DREB_v2_ila_in(89) <= asic_spi_mosi_int;
--DREB_v2_ila_in(90) <= ASPIC_ss_t_ccd_1_int;
--DREB_v2_ila_in(91) <= ASPIC_ss_b_ccd_1_int;
--DREB_v2_ila_in(92) <= aspic_miso_sel;
--DREB_v2_ila_in(93) <= asic_spi_miso_t;
--DREB_v2_ila_in(94) <= asic_spi_miso_b;
  -- DREB_v2_ila_in(69 downto 86) <=  bs_adc_data_from_adc_int;
  --DREB_v2_ila_in(71 downto 74) <=  ck_adc_data_from_adc_dcr;




 DREB_v2_ila_in(54) <= bs_adc_data_from_adc_int(0);
  DREB_v2_ila_in(55) <= bs_adc_data_from_adc_int(1);
  DREB_v2_ila_in(56) <= bs_adc_data_from_adc_int(2);
  DREB_v2_ila_in(57) <= bs_adc_data_from_adc_int(3);
  DREB_v2_ila_in(58) <= bs_adc_data_from_adc(4);
  DREB_v2_ila_in(59) <= bs_adc_data_from_adc(5);
  DREB_v2_ila_in(60) <= bs_adc_data_from_adc(6);
  DREB_v2_ila_in(61) <= bs_adc_data_from_adc(7);
  DREB_v2_ila_in(62) <= bs_adc_data_from_adc(8);
  DREB_v2_ila_in(63) <= bs_adc_data_from_adc(9);
  DREB_v2_ila_in(64) <= bs_adc_data_from_adc(10);
  DREB_v2_ila_in(65) <= bs_adc_data_from_adc(11);
  DREB_v2_ila_in(66) <= bs_adc_data_from_adc(12);
  DREB_v2_ila_in(67) <= bs_adc_data_from_adc(13);
  DREB_v2_ila_in(68) <= bs_adc_data_from_adc(14);
  DREB_v2_ila_in(69) <= bs_adc_data_from_adc(15);

  
  DREB_v2_ila_in(70) <= bs_adc_start_read;

  DREB_v2_ila_in(71) <= ck_adc_data_from_adc_int(0);
  DREB_v2_ila_in(72) <= ck_adc_data_from_adc_int(1);
  DREB_v2_ila_in(73) <= ck_adc_data_from_adc_int(2);
  DREB_v2_ila_in(74) <= ck_adc_data_from_adc_int(3);
  DREB_v2_ila_in(75) <= ck_adc_data_from_adc(4);
  DREB_v2_ila_in(76) <= ck_adc_data_from_adc(5);
  DREB_v2_ila_in(77) <= ck_adc_data_from_adc(6);
  DREB_v2_ila_in(78) <= ck_adc_data_from_adc(7);
  DREB_v2_ila_in(79) <= ck_adc_data_from_adc(8);
  DREB_v2_ila_in(80) <= ck_adc_data_from_adc(9);
  DREB_v2_ila_in(81) <= ck_adc_data_from_adc(10);
  DREB_v2_ila_in(82) <= ck_adc_data_from_adc(11);
  DREB_v2_ila_in(83) <= ck_adc_data_from_adc(12);
  DREB_v2_ila_in(84) <= ck_adc_data_from_adc(13);
  DREB_v2_ila_in(85) <= ck_adc_data_from_adc(14);
  DREB_v2_ila_in(86) <= ck_adc_data_from_adc(15);


  DREB_v2_ila_in(87) <= ck_adc_busy;
  DREB_v2_ila_in(88) <= ck_adc_EOC;
  DREB_v2_ila_in(89) <= ck_adc_CS_int;
  DREB_v2_ila_in(90) <= ck_adc_RD_int;
  DREB_v2_ila_in(91) <= ck_adc_WR_int;
  DREB_v2_ila_in(92) <= ck_adc_CONVST_int;
  DREB_v2_ila_in(93) <= ck_adc_SHDN_int;

  DREB_v2_ila_in(94) <= ck_adc_start_read;
  DREB_v2_ila_in(95) <= ck_adc_start_write;


--
--  DREB_v2_ila_in(89) <= CABAC_ss_t_ccd_1_int;
--  DREB_v2_ila_in(90) <= CABAC_ss_t_ccd_1_int;
--  DREB_v2_ila_in(91) <= CABAC_ss_b_ccd_1_int;
------DREB_v2_ila_in(92)                                          <= CABAC_mosi_int;
------DREB_v2_ila_in(93)                                          <= CABAC_sclk_int;
--  DREB_v2_ila_in(92) <= asic_spi_mosi_int;
--  DREB_v2_ila_in(93) <= asic_spi_sclk_int;
--  DREB_v2_ila_in(94) <= asic_spi_miso_t;
--  DREB_v2_ila_in(95) <= asic_spi_miso_b;
------DREB_v2_ila_in(94)                                          <= dataSOT;
--DREB_v2_ila_in(95)                                            <= dataEOT;

end Behavioral;

