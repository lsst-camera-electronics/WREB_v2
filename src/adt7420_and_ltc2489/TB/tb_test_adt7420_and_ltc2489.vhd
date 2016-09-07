--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   16:25:36 12/08/2014
-- Design Name:   
-- Module Name:   /home/srusso/Xilinx_prj/LSST_prj/test_adt7420_and_ltc2489/src/TB/tb_test_adt7420_and_ltc2489.vhd
-- Project Name:  test_adt7420_and_ltc2489
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: adt_7420_and_ltc2489_top
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
 
ENTITY tb_test_adt7420_and_ltc2489 IS
END tb_test_adt7420_and_ltc2489;
 
ARCHITECTURE behavior OF tb_test_adt7420_and_ltc2489 IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT adt_7420_and_ltc2489_top
    PORT(
         clk : IN  std_logic;
         reset : IN  std_logic;
         start_read_board_t : IN  std_logic;
         start_read_chip_t : IN  std_logic;
         read_chip_add : IN  std_logic_vector(1 downto 0);
         busy : OUT  std_logic;
         error_board_T1 : OUT  std_logic;
         board_T1_out : OUT  std_logic_vector(15 downto 0);
         error_board_T2 : OUT  std_logic;
         board_T2_out : OUT  std_logic_vector(15 downto 0);
         error_board_T3 : OUT  std_logic;
         board_T3_out : OUT  std_logic_vector(15 downto 0);
         error_board_T4 : OUT  std_logic;
         board_T4_out : OUT  std_logic_vector(15 downto 0);
         error_chip_t : OUT  std_logic;
         chip_t : OUT  std_logic_vector(23 downto 0);
         sda : INOUT  std_logic;
         scl : INOUT  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';
   signal reset : std_logic := '1';
   signal start_read_board_t : std_logic := '0';
   signal start_read_chip_t : std_logic := '0';
   signal read_chip_add : std_logic_vector(1 downto 0) := (others => '0');

	--BiDirs
   signal sda : std_logic;
   signal scl : std_logic;

 	--Outputs
   signal busy : std_logic;
   signal error_board_T1 : std_logic;
   signal board_T1_out : std_logic_vector(15 downto 0);
   signal error_board_T2 : std_logic;
   signal board_T2_out : std_logic_vector(15 downto 0);
   signal error_board_T3 : std_logic;
   signal board_T3_out : std_logic_vector(15 downto 0);
   signal error_board_T4 : std_logic;
   signal board_T4_out : std_logic_vector(15 downto 0);
   signal error_chip_t : std_logic;
   signal chip_t : std_logic_vector(23 downto 0);

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: adt_7420_and_ltc2489_top PORT MAP (
          clk => clk,
          reset => reset,
          start_read_board_t => start_read_board_t,
          start_read_chip_t => start_read_chip_t,
          read_chip_add => read_chip_add,
          busy => busy,
          error_board_T1 => error_board_T1,
          board_T1_out => board_T1_out,
          error_board_T2 => error_board_T2,
          board_T2_out => board_T2_out,
          error_board_T3 => error_board_T3,
          board_T3_out => board_T3_out,
          error_board_T4 => error_board_T4,
          board_T4_out => board_T4_out,
          error_chip_t => error_chip_t,
          chip_t => chip_t,
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
		
		wait for 1025 ns;
		start_read_board_t	<= '1';
		wait for 2*clk_period;
		start_read_board_t	<= '0';
		
		wait for 500 us;
		start_read_chip_t	<= '1';
		read_chip_add		<= "01";
		wait for 2*clk_period;
		start_read_chip_t	<= '0';
		
	
		
		
		

      wait;
   end process;

END;
