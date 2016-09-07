--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   18:22:57 02/19/2016
-- Design Name:   
-- Module Name:   /u1/srusso/Xilinx_prj/LSST_prj/test_max_11046/src/TB/tb_max11064_top.vhd
-- Project Name:  test_max_11046
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: max_11046_top
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
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY tb_max11064_top IS
END tb_max11064_top;
 
ARCHITECTURE behavior OF tb_max11064_top IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT max_11046_top
    PORT(
         clk : IN  std_logic;
         reset : IN  std_logic;
         start_write : IN  std_logic;
         start_read : IN  std_logic;
         EOC : IN  std_logic;
         data_to_adc : IN  std_logic_vector(3 downto 0);
         data_from_adc : IN  std_logic_vector(15 downto 0);
         link_busy : OUT  std_logic;
         CS : OUT  std_logic;
         RD : OUT  std_logic;
         WR : OUT  std_logic;
         CONVST : OUT  std_logic;
         SHDN : OUT  std_logic;
         data_to_adc_out : OUT  std_logic_vector(3 downto 0);
         cnv_results : OUT  std_logic_vector(15 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';
   signal reset : std_logic := '1';
   signal start_write : std_logic := '0';
   signal start_read : std_logic := '0';
   signal EOC : std_logic := '0';
   signal data_to_adc : std_logic_vector(3 downto 0) := (others => '0');
   signal data_from_adc : std_logic_vector(15 downto 0) := (others => '0');

 	--Outputs
   signal link_busy : std_logic;
   signal CS : std_logic;
   signal RD : std_logic;
   signal WR : std_logic;
   signal CONVST : std_logic;
   signal SHDN : std_logic;
   signal data_to_adc_out : std_logic_vector(3 downto 0);
   signal cnv_results : std_logic_vector(15 downto 0);

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: max_11046_top PORT MAP (
          clk => clk,
          reset => reset,
          start_write => start_write,
          start_read => start_read,
          EOC => EOC,
          data_to_adc => data_to_adc,
          data_from_adc => data_from_adc,
          link_busy => link_busy,
          CS => CS,
          RD => RD,
          WR => WR,
          CONVST => CONVST,
          SHDN => SHDN,
          data_to_adc_out => data_to_adc_out,
          cnv_results => cnv_results
        );

   -- Clock process definitions
   clk_process :process
   begin
		clk <= '0';
		wait for clk_period/2;
		clk <= '1';
		wait for clk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
      wait for 100 ns;	

      wait for clk_period*10;

      -- insert stimulus here 

      wait;
   end process;

END;
