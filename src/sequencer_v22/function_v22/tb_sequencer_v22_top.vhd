--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   19:01:40 04/10/2013
-- Design Name:   
-- Module Name:   /home/srusso/Xilinx_prj/LSST_prj/sequencer_v22/src/sequencer_v22/function_v22/tb_sequencer_v22_top.vhd
-- Project Name:  sequencer_v22
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: sequencer_v22_top
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
 
ENTITY tb_sequencer_v22_top IS
END tb_sequencer_v22_top;
 
ARCHITECTURE behavior OF tb_sequencer_v22_top IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT sequencer_v22_top
    PORT(
         reset : IN  std_logic;
         clk : IN  std_logic;
         start_sequence : IN  std_logic;
         program_mem_we : IN  std_logic;
         program_mem_w_add : IN  std_logic_vector(9 downto 0);
         program_mem_data_in : IN  std_logic_vector(31 downto 0);
         prog_mem_redbk : OUT  std_logic_vector(31 downto 0);
         time_mem_w_en : IN  std_logic;
         time_mem_in : IN  std_logic_vector(15 downto 0);
         time_mem_w_add : IN  std_logic_vector(7 downto 0);
         time_mem_readbk : OUT  std_logic_vector(15 downto 0);
         out_mem_w_en : IN  std_logic;
         out_mem_in : IN  std_logic_vector(31 downto 0);
         out_mem_w_add : IN  std_logic_vector(7 downto 0);
         out_mem_readbk : OUT  std_logic_vector(31 downto 0);
         stop_sequence : IN  std_logic;
         step_sequence : IN  std_logic;
         sequencer_out : OUT  std_logic_vector(31 downto 0);
         end_sequence : OUT  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal reset : std_logic := '1';
   signal clk : std_logic := '0';
   signal start_sequence : std_logic := '0';
   signal program_mem_we : std_logic := '0';
   signal program_mem_w_add : std_logic_vector(9 downto 0) := (others => '0');
   signal program_mem_data_in : std_logic_vector(31 downto 0) := (others => '0');
   signal time_mem_w_en : std_logic := '0';
   signal time_mem_in : std_logic_vector(15 downto 0) := (others => '0');
   signal time_mem_w_add : std_logic_vector(7 downto 0) := (others => '0');
   signal out_mem_w_en : std_logic := '0';
   signal out_mem_in : std_logic_vector(31 downto 0) := (others => '0');
   signal out_mem_w_add : std_logic_vector(7 downto 0) := (others => '0');
   signal stop_sequence : std_logic := '0';
   signal step_sequence : std_logic := '0';

 	--Outputs
   signal prog_mem_redbk : std_logic_vector(31 downto 0);
   signal time_mem_readbk : std_logic_vector(15 downto 0);
   signal out_mem_readbk : std_logic_vector(31 downto 0);
   signal sequencer_out : std_logic_vector(31 downto 0);
   signal end_sequence : std_logic;

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: sequencer_v22_top PORT MAP (
          reset => reset,
          clk => clk,
          start_sequence => start_sequence,
          program_mem_we => program_mem_we,
          program_mem_w_add => program_mem_w_add,
          program_mem_data_in => program_mem_data_in,
          prog_mem_redbk => prog_mem_redbk,
          time_mem_w_en => time_mem_w_en,
          time_mem_in => time_mem_in,
          time_mem_w_add => time_mem_w_add,
          time_mem_readbk => time_mem_readbk,
          out_mem_w_en => out_mem_w_en,
          out_mem_in => out_mem_in,
          out_mem_w_add => out_mem_w_add,
          out_mem_readbk => out_mem_readbk,
          stop_sequence => stop_sequence,
          step_sequence => step_sequence,
          sequencer_out => sequencer_out,
          end_sequence => end_sequence
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
		
		reset	<= '0';
		
		wait for 200 ns;
		
		-- set up sequencer
		--default state
		out_mem_w_en	<= '1';
		out_mem_in		<= x"ffff_ffff";
		out_mem_w_add	<= x"00";
		wait for clk_period*2;
		out_mem_w_en	<= '0';
		out_mem_in		<= x"0000_0000";
		out_mem_w_add	<= x"00";
		
		-- function 1
		-- time slice 0
		wait for clk_period*2;
		out_mem_w_en	<= '1';
		out_mem_in		<= x"1000_0032";
		out_mem_w_add	<= x"10";
		
		time_mem_w_en	<= '1';
		time_mem_in		<= x"0020";
		time_mem_w_add	<= x"10";
		wait for clk_period*2;
		out_mem_w_en	<= '0';
		out_mem_in		<= x"0000_0000";
		out_mem_w_add	<= x"00";
		time_mem_w_en	<= '0';
		time_mem_in		<= x"0000";
		time_mem_w_add	<= x"00";	

		-- time slice 1
		wait for clk_period*5;
		out_mem_w_en	<= '1';
		out_mem_in		<= x"1100_0010";
		out_mem_w_add	<= x"11";
		
		time_mem_w_en	<= '1';
		time_mem_in		<= x"000a";
		time_mem_w_add	<= x"11";
		wait for clk_period*2;
		out_mem_w_en	<= '0';
		out_mem_in		<= x"0000_0000";
		out_mem_w_add	<= x"00";
		time_mem_w_en	<= '0';
		time_mem_in		<= x"0000";
		time_mem_w_add	<= x"00";	
		
		-- time slice 2
		wait for clk_period*5;
		out_mem_w_en	<= '1';
		out_mem_in		<= x"1200_0032";
		out_mem_w_add	<= x"12";
		
		time_mem_w_en	<= '1';
		time_mem_in		<= x"0020";
		time_mem_w_add	<= x"12";
		wait for clk_period*2;
		out_mem_w_en	<= '0';
		out_mem_in		<= x"0000_0000";
		out_mem_w_add	<= x"00";
		time_mem_w_en	<= '0';
		time_mem_in		<= x"0000";
		time_mem_w_add	<= x"00";	
		
		-- time slice 3
		wait for clk_period*5;
		out_mem_w_en	<= '1';
		out_mem_in		<= x"1300_0010";
		out_mem_w_add	<= x"13";
		
		time_mem_w_en	<= '1';
		time_mem_in		<= x"000a";
		time_mem_w_add	<= x"13";
		wait for clk_period*2;
		out_mem_w_en	<= '0';
		out_mem_in		<= x"0000_0000";
		out_mem_w_add	<= x"00";
		time_mem_w_en	<= '0';
		time_mem_in		<= x"0000";
		time_mem_w_add	<= x"00";	
		
		
		-- function 3
		-- time slice 0		
		wait for clk_period*5;
		out_mem_w_en	<= '1';
		out_mem_in		<= x"3000_0032";
		out_mem_w_add	<= x"30";
		
		time_mem_w_en	<= '1';
		time_mem_in		<= x"0020";
		time_mem_w_add	<= x"30";
		wait for clk_period*2;
		out_mem_w_en	<= '0';
		out_mem_in		<= x"0000_0000";
		out_mem_w_add	<= x"00";
		time_mem_w_en	<= '0';
		time_mem_in		<= x"0000";
		time_mem_w_add	<= x"00";	
		
		-- time slice 1		
		wait for clk_period*5;
		out_mem_w_en	<= '1';
		out_mem_in		<= x"3100_0001";
		out_mem_w_add	<= x"31";
		
		time_mem_w_en	<= '1';
		time_mem_in		<= x"0001";
		time_mem_w_add	<= x"31";
		wait for clk_period*2;
		out_mem_w_en	<= '0';
		out_mem_in		<= x"0000_0000";
		out_mem_w_add	<= x"00";
		time_mem_w_en	<= '0';
		time_mem_in		<= x"0000";
		time_mem_w_add	<= x"00";	
		
		
		-- time slice 2		
		wait for clk_period*5;
		out_mem_w_en	<= '1';
		out_mem_in		<= x"3200_0002";
		out_mem_w_add	<= x"32";
		
		time_mem_w_en	<= '1';
		time_mem_in		<= x"0002";
		time_mem_w_add	<= x"32";
		wait for clk_period*2;
		out_mem_w_en	<= '0';
		out_mem_in		<= x"0000_0000";
		out_mem_w_add	<= x"00";
		time_mem_w_en	<= '0';
		time_mem_in		<= x"0000";
		time_mem_w_add	<= x"00";	
		
		-- time slice 3		
		wait for clk_period*5;
		out_mem_w_en	<= '1';
		out_mem_in		<= x"3300_0010";
		out_mem_w_add	<= x"33";
		
		time_mem_w_en	<= '1';
		time_mem_in		<= x"000a";
		time_mem_w_add	<= x"33";
		wait for clk_period*2;
		out_mem_w_en	<= '0';
		out_mem_in		<= x"0000_0000";
		out_mem_w_add	<= x"00";
		time_mem_w_en	<= '0';
		time_mem_in		<= x"0000";
		time_mem_w_add	<= x"00";	
		
		
		-- write program memory
		
		wait for 100 ns;
		
		
		
		wait for 100 ns;
		program_mem_we <= '1';
		program_mem_w_add	<= "00" & x"00";
		program_mem_data_in <= x"11000002";
		
		wait for clk_period;
		program_mem_we <= '1';
		program_mem_w_add	<= "00" & x"01";
		program_mem_data_in <= x"01000002";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"02";
		program_mem_data_in <= x"13000003";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"03";
		program_mem_data_in <= x"11000001";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"04";
		program_mem_data_in <= x"20280002";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"05";
		program_mem_data_in <= x"40000000";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"0A";
		program_mem_data_in <= x"20340002";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"0B";
		program_mem_data_in <= x"11000003";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"0C";
		program_mem_data_in <= x"30000000";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"0D";
		program_mem_data_in <= x"13000001";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"0E";
		program_mem_data_in <= x"11000002";
		
		wait for clk_period;
		program_mem_w_add	<= "00" & x"0F";
		program_mem_data_in <= x"30000000";
		
		wait for clk_period;
		program_mem_we <= '0';
		program_mem_w_add	<= "00" & x"00";
		program_mem_data_in <= x"00000000";
		
		-- sequencer trigger
		wait for clk_period*20;
		start_sequence <= '1';
		
		wait for clk_period*2;
		start_sequence <= '0';
		
		wait for 10 us;
		
		step_sequence <= '1';
		wait for clk_period*2;
		step_sequence <= '0';

      wait;
   end process;

END;
