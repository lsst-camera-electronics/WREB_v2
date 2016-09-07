--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   18:04:00 10/29/2014
-- Design Name:   
-- Module Name:   /home/srusso/Xilinx_prj/LSST_prj/test_cabac_1_spi_link_top_mux/src/TB/tb_test_cabac_1_spi_link_top_mux.vhd
-- Project Name:  test_cabac_1_spi_link_top_mux
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: CABAC_1_spi_link_top_mux
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
 
ENTITY tb_test_cabac_1_spi_link_top_mux IS
END tb_test_cabac_1_spi_link_top_mux;
 
ARCHITECTURE behavior OF tb_test_cabac_1_spi_link_top_mux IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT CABAC_1_spi_link_top_mux
    PORT(
         clk : IN  std_logic;
         reset : IN  std_logic;
         start_link_trans : IN  std_logic;
         start_reset : IN  std_logic;
         miso_ccd1 : IN  std_logic;
         miso_ccd2 : IN  std_logic;
         miso_ccd3 : IN  std_logic;
         word2send : IN  std_logic_vector(31 downto 0);
         cabac_mosi : OUT  std_logic;
         ss_t_ccd1 : OUT  std_logic;
         ss_t_ccd2 : OUT  std_logic;
         ss_t_ccd3 : OUT  std_logic;
         ss_b_ccd1 : OUT  std_logic;
         ss_b_ccd2 : OUT  std_logic;
         ss_b_ccd3 : OUT  std_logic;
         cabac_sclk : OUT  std_logic;
         cabac_n_reset : OUT  std_logic;
         busy : OUT  std_logic;
         d_slave_ready_ccd1 : OUT  std_logic;
         d_slave_ready_ccd2 : OUT  std_logic;
         d_slave_ready_ccd3 : OUT  std_logic;
         d_from_slave_ccd1 : OUT  std_logic_vector(15 downto 0);
         d_from_slave_ccd2 : OUT  std_logic_vector(15 downto 0);
         d_from_slave_ccd3 : OUT  std_logic_vector(15 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';
   signal reset : std_logic := '1';
   signal start_link_trans : std_logic := '0';
   signal start_reset : std_logic := '0';
   signal miso_ccd1 : std_logic := '0';
   signal miso_ccd2 : std_logic := '0';
   signal miso_ccd3 : std_logic := '0';
   signal word2send : std_logic_vector(31 downto 0) := (others => '0');

 	--Outputs
   signal cabac_mosi : std_logic;
   signal ss_t_ccd1 : std_logic;
   signal ss_t_ccd2 : std_logic;
   signal ss_t_ccd3 : std_logic;
   signal ss_b_ccd1 : std_logic;
   signal ss_b_ccd2 : std_logic;
   signal ss_b_ccd3 : std_logic;
   signal cabac_sclk : std_logic;
   signal cabac_n_reset : std_logic;
   signal busy : std_logic;
   signal d_slave_ready_ccd1 : std_logic;
   signal d_slave_ready_ccd2 : std_logic;
   signal d_slave_ready_ccd3 : std_logic;
   signal d_from_slave_ccd1 : std_logic_vector(15 downto 0);
   signal d_from_slave_ccd2 : std_logic_vector(15 downto 0);
   signal d_from_slave_ccd3 : std_logic_vector(15 downto 0);
	
	-- internal 
	signal read_write : std_logic := '0';
   signal reg_add 	: std_logic_vector(6 downto 0) := (others => '0');
   signal d_to_slave : std_logic_vector(15 downto 0) := (others => '0');
	
	signal write_ccd1	: std_logic := '0';
	signal write_ccd2	: std_logic := '0';
	signal write_ccd3	: std_logic := '0';
	signal cabac_top	: std_logic := '0';
	signal cabac_bot	: std_logic := '0';
	
	signal reset_ccd1	: std_logic := '0';
	signal reset_ccd2	: std_logic := '0';
	signal reset_ccd3	: std_logic := '0';

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: CABAC_1_spi_link_top_mux PORT MAP (
          clk => clk,
          reset => reset,
          start_link_trans => start_link_trans,
          start_reset => start_reset,
          miso_ccd1 => miso_ccd1,
          miso_ccd2 => miso_ccd2,
          miso_ccd3 => miso_ccd3,
          word2send => word2send,
          cabac_mosi => cabac_mosi,
          ss_t_ccd1 => ss_t_ccd1,
          ss_t_ccd2 => ss_t_ccd2,
          ss_t_ccd3 => ss_t_ccd3,
          ss_b_ccd1 => ss_b_ccd1,
          ss_b_ccd2 => ss_b_ccd2,
          ss_b_ccd3 => ss_b_ccd3,
          cabac_sclk => cabac_sclk,
          cabac_n_reset => cabac_n_reset,
          busy => busy,
          d_slave_ready_ccd1 => d_slave_ready_ccd1,
          d_slave_ready_ccd2 => d_slave_ready_ccd2,
          d_slave_ready_ccd3 => d_slave_ready_ccd3,
          d_from_slave_ccd1 => d_from_slave_ccd1,
          d_from_slave_ccd2 => d_from_slave_ccd2,
          d_from_slave_ccd3 => d_from_slave_ccd3
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

        	reset <= '0';

      wait for clk_period*10;

      -- insert stimulus here 


		read_write	<= '1';
		reg_add		<= "0000001";
		d_to_slave	<= x"0003";
		write_ccd1	<= '1';
		write_ccd2	<= '0';
		write_ccd3	<= '0';
		cabac_top	<= '1';
		cabac_bot	<= '0';
		
		wait for clk_period;
		
		word2send(31 downto 29)	<= "100";
		word2send(28)				<= write_ccd3;
		word2send(27)				<= write_ccd2;
		word2send(26)				<= write_ccd1;
		word2send(25)				<= cabac_bot;
		word2send(24)				<= cabac_top;
		word2send(23)				<= read_write;
		word2send(22 downto 16)	<= reg_add;
		word2send(15 downto 0)	<= d_to_slave;
		
		wait for clk_period*5;
		start_link_trans		<= '1';
		
		wait for clk_period*1;
		start_link_trans 		<= '0';
		
		wait for 3 us;
		
		
		read_write	<= '1';
		reg_add		<= "0001001";
		d_to_slave	<= x"ffff";
		write_ccd1	<= '1';
		write_ccd2	<= '0';
		write_ccd3	<= '0';
		cabac_top	<= '1';
		cabac_bot	<= '1';
		
		wait for clk_period;
		
		word2send(31 downto 29)	<= "100";
		word2send(28)				<= write_ccd3;
		word2send(27)				<= write_ccd2;
		word2send(26)				<= write_ccd1;
		word2send(25)				<= cabac_bot;
		word2send(24)				<= cabac_top;
		word2send(23)				<= read_write;
		word2send(22 downto 16)	<= reg_add;
		word2send(15 downto 0)	<= d_to_slave;
		
		wait for clk_period*5;
		start_link_trans		<= '1';
		
		wait for clk_period*1;
		start_link_trans 		<= '0';
		
		wait for 3 us;
		
		wait for clk_period*5;
		start_reset			<= '1';
		wait for clk_period*2;
		start_reset		<= '0';
		
		wait for 3 us;
	


      wait;
   end process;

END;
