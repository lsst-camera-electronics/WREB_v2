--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   17:16:14 10/31/2014
-- Design Name:   
-- Module Name:   /home/srusso/Xilinx_prj/LSST_prj/ltc2945_multi_read_wreb/src/TB/TB_ltc2945_multi_read_top_wreb.vhd
-- Project Name:  ltc2945_multi_read_wreb
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: ltc2945_multi_read_top_wreb
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
 
ENTITY TB_ltc2945_multi_read_top_wreb IS
END TB_ltc2945_multi_read_top_wreb;
 
ARCHITECTURE behavior OF TB_ltc2945_multi_read_top_wreb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT ltc2945_multi_read_top_wreb
    PORT(
         clk : IN  std_logic;
         reset : IN  std_logic;
         start_procedure : IN  std_logic;
         busy : OUT  std_logic;
         error_V_HTR_voltage : OUT  std_logic;
         V_HTR_voltage_out : OUT  std_logic_vector(15 downto 0);
         error_V_HTR_current : OUT  std_logic;
         V_HTR_current_out : OUT  std_logic_vector(15 downto 0);
         error_V_DREB_voltage : OUT  std_logic;
         V_DREB_voltage_out : OUT  std_logic_vector(15 downto 0);
         error_V_DREB_current : OUT  std_logic;
         V_DREB_current_out : OUT  std_logic_vector(15 downto 0);
         error_V_CLK_H_voltage : OUT  std_logic;
         V_CLK_H_voltage_out : OUT  std_logic_vector(15 downto 0);
         error_V_CLK_H_current : OUT  std_logic;
         V_CLK_H_current_out : OUT  std_logic_vector(15 downto 0);
         error_V_DPHI_voltage : OUT  std_logic;
         V_DPHI_voltage_out : OUT  std_logic_vector(15 downto 0);
         error_V_DPHI_current : OUT  std_logic;
         V_DPHI_current_out : OUT  std_logic_vector(15 downto 0);
         error_V_ANA_voltage : OUT  std_logic;
         V_ANA_voltage_out : OUT  std_logic_vector(15 downto 0);
         error_V_ANA_current : OUT  std_logic;
         V_ANA_current_out : OUT  std_logic_vector(15 downto 0);
         error_V_OD_voltage : OUT  std_logic;
         V_OD_voltage_out : OUT  std_logic_vector(15 downto 0);
         error_V_OD_current : OUT  std_logic;
         V_OD_current_out : OUT  std_logic_vector(15 downto 0);
         sda : INOUT  std_logic;
         scl : INOUT  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';
   signal reset : std_logic := '1';
   signal start_procedure : std_logic := '0';

	--BiDirs
   signal sda : std_logic := 'H';
   signal scl : std_logic := 'H';

 	--Outputs
   signal busy : std_logic;
   signal error_V_HTR_voltage : std_logic;
   signal V_HTR_voltage_out : std_logic_vector(15 downto 0);
   signal error_V_HTR_current : std_logic;
   signal V_HTR_current_out : std_logic_vector(15 downto 0);
   signal error_V_DREB_voltage : std_logic;
   signal V_DREB_voltage_out : std_logic_vector(15 downto 0);
   signal error_V_DREB_current : std_logic;
   signal V_DREB_current_out : std_logic_vector(15 downto 0);
   signal error_V_CLK_H_voltage : std_logic;
   signal V_CLK_H_voltage_out : std_logic_vector(15 downto 0);
   signal error_V_CLK_H_current : std_logic;
   signal V_CLK_H_current_out : std_logic_vector(15 downto 0);
   signal error_V_DPHI_voltage : std_logic;
   signal V_DPHI_voltage_out : std_logic_vector(15 downto 0);
   signal error_V_DPHI_current : std_logic;
   signal V_DPHI_current_out : std_logic_vector(15 downto 0);
   signal error_V_ANA_voltage : std_logic;
   signal V_ANA_voltage_out : std_logic_vector(15 downto 0);
   signal error_V_ANA_current : std_logic;
   signal V_ANA_current_out : std_logic_vector(15 downto 0);
   signal error_V_OD_voltage : std_logic;
   signal V_OD_voltage_out : std_logic_vector(15 downto 0);
   signal error_V_OD_current : std_logic;
   signal V_OD_current_out : std_logic_vector(15 downto 0);

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: ltc2945_multi_read_top_wreb PORT MAP (
          clk => clk,
          reset => reset,
          start_procedure => start_procedure,
          busy => busy,
          error_V_HTR_voltage => error_V_HTR_voltage,
          V_HTR_voltage_out => V_HTR_voltage_out,
          error_V_HTR_current => error_V_HTR_current,
          V_HTR_current_out => V_HTR_current_out,
          error_V_DREB_voltage => error_V_DREB_voltage,
          V_DREB_voltage_out => V_DREB_voltage_out,
          error_V_DREB_current => error_V_DREB_current,
          V_DREB_current_out => V_DREB_current_out,
          error_V_CLK_H_voltage => error_V_CLK_H_voltage,
          V_CLK_H_voltage_out => V_CLK_H_voltage_out,
          error_V_CLK_H_current => error_V_CLK_H_current,
          V_CLK_H_current_out => V_CLK_H_current_out,
          error_V_DPHI_voltage => error_V_DPHI_voltage,
          V_DPHI_voltage_out => V_DPHI_voltage_out,
          error_V_DPHI_current => error_V_DPHI_current,
          V_DPHI_current_out => V_DPHI_current_out,
          error_V_ANA_voltage => error_V_ANA_voltage,
          V_ANA_voltage_out => V_ANA_voltage_out,
          error_V_ANA_current => error_V_ANA_current,
          V_ANA_current_out => V_ANA_current_out,
          error_V_OD_voltage => error_V_OD_voltage,
          V_OD_voltage_out => V_OD_voltage_out,
          error_V_OD_current => error_V_OD_current,
          V_OD_current_out => V_OD_current_out,
          sda => sda,
          scl => scl
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
		
		reset		<= '0';
		sda		<= 'H';
		scl		<='H';
		
		wait for 1 us;
		
		start_procedure		<= '1';
		wait for clk_period;
		start_procedure		<= '0';

      wait;
   end process;

END;
