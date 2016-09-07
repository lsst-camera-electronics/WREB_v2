----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    17:09:59 06/21/2012 
-- Design Name: 
-- Module Name:    sequencer_paramete_extractor - Behavioral 
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
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity sequencer_parameter_extractor_top is

port (
		clk 						: in  std_logic;
		reset						: in  std_logic;
		start_sequence			: in  std_logic;
		program_mem_we			: in  std_logic;
		program_mem_w_add		: in  std_logic_vector(9 downto 0);
		program_mem_data_in	: in  std_logic_vector(31 downto 0);
		
		
		fifo_param_re			: in  std_logic;
		
		prog_mem_redbk			: out std_logic_vector(31 downto 0);
		fifo_param_empty		: out std_logic;
		fifo_param_out			: out std_logic_vector(28 downto 0)		
		);

end sequencer_parameter_extractor_top;

architecture Behavioral of sequencer_parameter_extractor_top is


component parameter_extractor_fsm is
port (
		clk 					: in  std_logic;
		reset					: in  std_logic;
		start_sequence		: in  std_logic;
		fifo_param_full	: in  std_logic;
		program_mem_data	: in  std_logic_vector(31 downto 0);
		data_from_stack	: in  std_logic_vector(31 downto 0);
		fifo_param_write	: out std_logic;
		sub_stack_w_en		: out std_logic;
		sub_stack_add		: out std_logic_vector(3  downto 0);
		sub_rep_cnt			: out std_logic_vector(16 downto 0);
		program_mem_add	: out std_logic_vector(9 downto 0)
		);
end component;

component generic_single_port_ram is
generic( 
			data_width : integer;
			add_width 	: integer);
port (			
		clk      		: in  std_logic;           -- clock
		ram_wr_en 		: in  std_logic;	 			-- data in
		ram_add 			: in  std_logic_vector(add_width-1 downto 0);
		ram_data_in		: in  std_logic_vector(data_width-1 downto 0);
		ram_data_out	: out std_logic_vector(data_width-1 downto 0));          -- data out
end component;

component generic_dual_port_ram is
generic( 
			data_width : integer := 32;
			add_width 	: integer := 8);
port (
		clk      		: in  std_logic;           -- clock
		ram_wr_en 		: in  std_logic;	 			-- data in
		ram_wr_add 	: in  std_logic_vector(add_width-1 downto 0);
		ram_rd_add 	: in  std_logic_vector(add_width-1 downto 0);
		ram_data_in	: in  std_logic_vector(data_width-1 downto 0);
		ram_data_out_1	: out std_logic_vector(data_width-1 downto 0);
		ram_data_out_2	: out std_logic_vector(data_width-1 downto 0)
		);          
end component;

COMPONENT seq_param_fifo
  PORT (
    clk : IN STD_LOGIC;
    srst : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(28 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(28 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC
  );
END COMPONENT;


signal fifo_param_full	: std_logic;
signal prog_mem_data_out: std_logic_vector(31 downto 0);
signal data_from_stack	: std_logic_vector(27 downto 0);
signal fifo_param_we		: std_logic;
signal sub_stack_w_en	: std_logic;
signal sub_stack_add		: std_logic_vector(3 downto 0);
signal program_mem_rd_add	: std_logic_vector(9 downto 0);
signal stack_data_in		: std_logic_vector(27 downto 0);
signal sub_rep_cnt			: std_logic_vector(16 downto 0);
signal fifo_din			: std_logic_vector(28 downto 0);


begin

parameter_extractor_fsm_0 : parameter_extractor_fsm 
port map (
		clk 						=> clk,
		reset						=> reset,
		start_sequence			=> start_sequence,
		fifo_param_full		=> fifo_param_full,
		program_mem_data		=> prog_mem_data_out,
		data_from_stack(31 downto 28)		=> x"0",
		data_from_stack(27 downto 0)		=>data_from_stack,
		fifo_param_write		=> fifo_param_we,
		sub_stack_w_en			=> sub_stack_w_en,
		sub_stack_add			=> sub_stack_add,
		sub_rep_cnt				=> sub_rep_cnt,
		program_mem_add		=> program_mem_rd_add
		);

function_stack : generic_single_port_ram 
generic map( 
			data_width => 28,
			add_width  => 4)
port map (			
		clk      		=> clk,
		ram_wr_en 		=> sub_stack_w_en,	 			
		ram_add 			=> sub_stack_add,
		ram_data_in		=> stack_data_in,
		ram_data_out	=> data_from_stack); 
		
program_memory : generic_dual_port_ram
generic map ( 
			data_width => 32,
			add_width  => 10)
port map (
		clk      		=> clk,
		ram_wr_en 		=> program_mem_we,
		ram_wr_add 		=> program_mem_w_add,
		ram_rd_add 		=> program_mem_rd_add,
		ram_data_in		=> program_mem_data_in,
		ram_data_out_1	=> prog_mem_data_out,
		ram_data_out_2	=> prog_mem_redbk); 
		
seq_param_fifo_0 : seq_param_fifo
  PORT MAP (
    clk 		=> clk,
    srst 	=> reset,
    din 		=> fifo_din,
    wr_en 	=> fifo_param_we,
    rd_en 	=> fifo_param_re,
    dout 	=> fifo_param_out,
    full 	=> fifo_param_full,
    empty 	=> fifo_param_empty
  );


fifo_din		<= prog_mem_data_out(30) & prog_mem_data_out(27 downto 0);

--stack_data_in <= program_mem_rd_add & prog_mem_data_out(17 downto 0);
stack_data_in <= program_mem_rd_add & '0' & sub_rep_cnt;
end Behavioral;

