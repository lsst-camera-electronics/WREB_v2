--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   16:01:30 12/12/2014
-- Design Name:   
-- Module Name:   /home/srusso/Xilinx_prj/LSST_prj/test_ad9628_dual_fast_adc/src/TB/tb_ad9628_dual_fast_adc.vhd
-- Project Name:  test_ad9628_dual_fast_adc
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: ad9628_dual_fast_adc_top
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
use work.ad9628_dual_fast_adc_package.all;

 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY tb_ad9628_dual_fast_adc IS
END tb_ad9628_dual_fast_adc;
 
ARCHITECTURE behavior OF tb_ad9628_dual_fast_adc IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT ad9628_dual_fast_adc_top
    PORT(
         clk : IN  std_logic;
         reset : IN  std_logic;
         start : IN  std_logic;
         num_data_to_read_en : IN  std_logic;
         num_data_to_read : IN  std_logic_vector(23 downto 0);
         adc_data_in_cha : IN  std_logic_vector(11 downto 0);
         adc_data_in_chb : IN  std_logic_vector(11 downto 0);
         dcoa : IN  std_logic;
         dcob : IN  std_logic;
         adc_clk_en : OUT  std_logic;
         busy : OUT  std_logic;
         adc_pdown : OUT  std_logic;
         write_en_sci : OUT  std_logic;
         adc_data_SOF : OUT  std_logic;
         adc_data_EOF : OUT  std_logic;
         adc_data_out_cha : OUT  std_logic_vector(11 downto 0);
         adc_data_out_chb : OUT  std_logic_vector(11 downto 0);
         out_fast_adc_reg : OUT  array1724
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';
   signal reset : std_logic := '1';
   signal start : std_logic := '0';
   signal num_data_to_read_en : std_logic := '0';
   signal num_data_to_read : std_logic_vector(23 downto 0) := (others => '0');
   signal adc_data_in_cha : std_logic_vector(11 downto 0) := (others => '0');
   signal adc_data_in_chb : std_logic_vector(11 downto 0) := (others => '0');
   signal dcoa : std_logic := '0';
   signal dcob : std_logic := '0';

 	--Outputs
   signal adc_clk_en : std_logic;
   signal busy : std_logic;
   signal adc_pdown : std_logic;
   signal write_en_sci : std_logic;
   signal adc_data_SOF : std_logic;
   signal adc_data_EOF : std_logic;
   signal adc_data_out_cha : std_logic_vector(11 downto 0);
   signal adc_data_out_chb : std_logic_vector(11 downto 0);
   signal out_fast_adc_reg : array1724;

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: ad9628_dual_fast_adc_top PORT MAP (
          clk => clk,
          reset => reset,
          start => start,
          num_data_to_read_en => num_data_to_read_en,
          num_data_to_read => num_data_to_read,
          adc_data_in_cha => adc_data_in_cha,
          adc_data_in_chb => adc_data_in_chb,
          dcoa => dcoa,
          dcob => dcob,
          adc_clk_en => adc_clk_en,
          busy => busy,
          adc_pdown => adc_pdown,
          write_en_sci => write_en_sci,
          adc_data_SOF => adc_data_SOF,
          adc_data_EOF => adc_data_EOF,
          adc_data_out_cha => adc_data_out_cha,
          adc_data_out_chb => adc_data_out_chb,
          out_fast_adc_reg => out_fast_adc_reg
        );

   -- Clock process definitions
   clk_process :process
   begin
		clk <= '0';
		wait for clk_period/2;
		clk <= '1';
		wait for clk_period/2;
   end process;
 
	 dcoa_dcob :process
   begin
--		if adc_clk_en = '1' then
			dcoa <= '1';
			dcob <= '1';
			wait for clk_period/2;
			dcoa <= '0';
			dcob <= '0';
			wait for clk_period/2;
--		else		
--			dcoa <= '1';
--			dcob <= '1';
--		end if;
   end process;

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
      wait for 100 ns;	
		
      -- insert stimulus here 
		
		      wait for 100 ns;	

      wait for clk_period*10;

      -- insert stimulus here 
		
		reset 					<= '0';
		adc_data_in_cha		<= x"fff";
		adc_data_in_chb		<= x"f0f";
		
		wait for 1 us;
		num_data_to_read_en 	<= '1';
		num_data_to_read		<= x"00000f";
		
		wait for clk_period;
		num_data_to_read_en 	<= '0';
		
		wait for 100 ns;
		
		start 			<= '1';
		wait for clk_period;
		start 			<= '0';


      wait;
   end process;

END;
