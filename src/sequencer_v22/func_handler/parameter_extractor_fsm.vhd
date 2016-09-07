----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    15:31:10 06/20/2012 
-- Design Name: 
-- Module Name:    parameter_extractor_fsm - Behavioral 
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

entity parameter_extractor_fsm is

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

end parameter_extractor_fsm;

architecture Behavioral of parameter_extractor_fsm is

	type   state_type is (wait_start, op_code_eval, sub_jump, simple_func_op, trailer_op, rep_sub);
	signal pres_state, next_state : state_type;
	signal next_fifo_param_write	: std_logic;
	signal next_sub_stack_w_en 	: std_logic;
	signal next_sub_stack_add		: std_logic_vector(3  downto 0);
	signal next_program_mem_add	: std_logic_vector(9 downto 0);
	signal next_sub_rep_cnt			: std_logic_vector(16 downto 0);
	
	
	
	signal sub_rep_cnt_int				: std_logic_vector(16 downto 0);
	signal sub_stack_add_int		: std_logic_vector(3 downto 0);
	signal program_mem_add_int		: std_logic_vector(9 downto 0);
	signal op_code						: std_logic_vector(3 downto 0);

	

begin


  process (clk)
  begin
    if clk'event and clk = '1' then
      if reset = '1' then
			pres_state 			<= wait_start;
			fifo_param_write	<= '0';
			sub_stack_w_en		<= '0';
			sub_stack_add_int		<= (others => '0'); 
			program_mem_add_int	<= (others => '0'); 
			sub_rep_cnt_int			<= (others => '0'); 
        
      else
			pres_state 			<= next_state;
			fifo_param_write	<= next_fifo_param_write;
			sub_stack_w_en		<= next_sub_stack_w_en;
			sub_stack_add_int		<= next_sub_stack_add; 
			program_mem_add_int	<= next_program_mem_add; 
			sub_rep_cnt_int  		<= next_sub_rep_cnt;

      end if;
    end if;
  end process;

  process (pres_state, start_sequence, op_code, program_mem_data, data_from_stack, fifo_param_full)
  begin

    -------------------- outputs defoult values --------------------
    
    next_fifo_param_write 	<= '0';
    next_sub_stack_w_en    <= '0';
    next_sub_stack_add   	<= sub_stack_add_int;
    next_program_mem_add 	<= program_mem_add_int;
	 next_sub_rep_cnt			<= sub_rep_cnt_int;

    case pres_state is
    
		when wait_start =>
			if start_sequence = '0' then
					next_state 				<= wait_start;
					next_sub_stack_add 	<= (others => '0');
					next_program_mem_add	<= (others => '0');
					next_sub_rep_cnt				<= (others => '0');
			else 
					next_state 	<= op_code_eval;
			end if;
			
		when op_code_eval => 
			if fifo_param_full = '1' then 
				next_state 	<= op_code_eval;
			else
				if op_code = x"1" then
					next_state 					<= simple_func_op;
					next_fifo_param_write	<= '1';
			elsif op_code = x"2" then
					next_state 				<= sub_jump;
					next_sub_rep_cnt		<= sub_rep_cnt_int + 1;
					next_sub_stack_w_en 	<= '1';
			elsif op_code = x"3" then
					next_state <= trailer_op;
					next_sub_stack_add	<= sub_stack_add_int - 1;
			elsif op_code = x"4" then
					next_state <= wait_start;
					next_fifo_param_write	<= '1'; -- test return to zero
			else
					next_state <= simple_func_op;
			end if;		
		end if;
		
		when simple_func_op => 
			next_state <= op_code_eval;
			next_program_mem_add		<= program_mem_add_int + 1;
			
		when sub_jump => 
			next_state <= op_code_eval;
			next_program_mem_add <= program_mem_data(27 downto 18);
			next_sub_stack_add	<= sub_stack_add_int + 1;
			next_sub_rep_cnt		<= (others => '0');

			
		when trailer_op => 
			next_state 				<= rep_sub;
			next_program_mem_add	<= data_from_stack(27 downto 18);
			
			when rep_sub => 
			if program_mem_data(16 downto 0) = data_from_stack(16 downto 0) then
					next_state 				<= simple_func_op;
					next_sub_rep_cnt		<= (others => '0');
		else 
					next_state 				<= op_code_eval;
					next_sub_rep_cnt		<= data_from_stack(16 downto 0);
		end if;
				
		 end case;
end process;		

sub_stack_add		<= sub_stack_add_int; 
sub_rep_cnt			<= sub_rep_cnt_int;
program_mem_add	<= program_mem_add_int;
op_code <= program_mem_data(31 downto 28);	
	
end Behavioral;

