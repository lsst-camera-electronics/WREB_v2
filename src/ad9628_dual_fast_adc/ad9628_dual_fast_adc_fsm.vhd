----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    16:42:17 12/11/2014 
-- Design Name: 
-- Module Name:    ad9628_dual_fast_adc_fsm - Behavioral 
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

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity ad9628_dual_fast_adc_fsm is

generic (pdown_reco_time	: integer := 35000);

port (
		clk						: in  std_logic;
		reset						: in  std_logic;
		start						: in  std_logic;
		fifo_empty				: in  std_logic;
		num_data_to_read		: in  std_logic_vector(23 downto 0);
		busy						: out std_logic;
		adc_pdown				: out std_logic;
		adc_clk_en				: out std_logic;
		fifo_read_en			: out std_logic;
		fifo_write_en			: out std_logic;
		write_en_sci			: out std_logic;
		adc_data_SOF			: out std_logic;
		adc_data_EOF			: out std_logic
		);

end ad9628_dual_fast_adc_fsm;

architecture Behavioral of ad9628_dual_fast_adc_fsm is
type state_type is (wait_start, 
							nap_mode_exit, 
							read_first_word,
							read_data, wait_data_fifo, emptying_fifo
							); 
							
signal pres_state, next_state : state_type;
signal next_busy						: std_logic;
signal next_adc_pdown				: std_logic;						
signal next_adc_clk_en				: std_logic;						
signal next_fifo_read_en			: std_logic;
signal next_fifo_write_en			: std_logic;
signal next_write_en_sci			: std_logic;						
signal next_adc_data_SOF			: std_logic;						
signal next_adc_data_EOF			: std_logic;
signal num_data_to_read_integer	: integer range 0 to 16777215;					

signal next_pdown_reco_cnt		: integer range 0 to pdown_reco_time;
signal next_data_cnt				: integer range 0 to 16777215;	
signal pdown_reco_cnt			: integer range 0 to pdown_reco_time;
signal data_cnt					: integer range 0 to 16777215;	
signal num_data_to_read_int	: integer range 0 to 16777215;

begin


process (clk)
  begin
    if clk'event and clk = '1' then
      if reset = '1' then
			pres_state   			<= wait_start;
			busy						<= '0';
			adc_pdown				<= '0';
			adc_clk_en				<= '0';
			fifo_read_en			<= '0';
			fifo_write_en			<= '0';
			write_en_sci			<= '0';
			adc_data_SOF			<= '0';
			adc_data_EOF			<= '0';
			pdown_reco_cnt			<= 0;
			data_cnt					<= 0;
		else
			pres_state   			<= next_state;
			busy						<= next_busy;
			adc_pdown				<= next_adc_pdown;
			adc_clk_en				<= next_adc_clk_en;
			fifo_read_en			<= next_fifo_read_en;
			fifo_write_en			<= next_fifo_write_en;
			write_en_sci			<= next_write_en_sci;
			adc_data_SOF			<= next_adc_data_SOF;
			adc_data_EOF			<= next_adc_data_EOF;
			pdown_reco_cnt			<= next_pdown_reco_cnt;
			data_cnt					<= next_data_cnt;
		end if;
	end if;
end process;

num_data_to_read_integer <= to_integer(signed(num_data_to_read));

 process (pres_state, start, fifo_empty, pdown_reco_cnt, data_cnt, num_data_to_read_integer)
  begin


    -------------------- outputs default values  --------------------
	 
			next_busy					<= '1';
			next_adc_pdown				<= '0';
			next_adc_clk_en			<= '1';
			next_fifo_read_en			<= '0';
			next_fifo_write_en		<= '0';
			next_write_en_sci			<= '0';
			next_adc_data_SOF			<= '0';
			next_adc_data_EOF			<= '0';
			next_pdown_reco_cnt		<= pdown_reco_cnt;
			next_data_cnt				<= data_cnt;

	case pres_state is

		when wait_start =>
			if start = '1' then 
				next_state 		<= nap_mode_exit;
			else 
				next_state			<= wait_start;
				next_busy			<= '0';
				next_adc_pdown		<= '1';
				next_adc_clk_en	<= '0';
			end if;
			
		when nap_mode_exit => 
			if pdown_reco_cnt = pdown_reco_time then
				next_state				<= read_first_word;
				next_fifo_write_en	<= '1';
				next_pdown_reco_cnt	<= 0;
			else
				next_state				<= nap_mode_exit;
				next_pdown_reco_cnt	<= pdown_reco_cnt + 1;
			end if;
			
		when read_first_word => 
			if fifo_empty = '0' then
				next_state				<= wait_data_fifo; 
				next_fifo_write_en	<= '1';
				next_fifo_read_en		<= '1';
				next_write_en_sci		<= '1';
				next_adc_data_SOF		<= '1';
				next_data_cnt			<= data_cnt + 1;
			else
				next_state				<= read_first_word;
				next_fifo_write_en	<= '1';
			end if;
			
		when wait_data_fifo => 
			if fifo_empty = '0' then
				next_state				<= read_data; 
				next_fifo_write_en	<= '1';
				next_fifo_read_en		<= '1';
				next_write_en_sci		<= '1';
				next_data_cnt			<= data_cnt + 1;
			else
				next_state				<= wait_data_fifo;
				next_fifo_write_en	<= '1';
			end if;
			
		when read_data => 
			if data_cnt = num_data_to_read_integer then
				next_state				<= emptying_fifo;
				next_adc_data_EOF		<= '1';
				next_fifo_write_en	<= '1';
				next_fifo_read_en		<= '1';
				next_write_en_sci		<= '1';
			else
				if fifo_empty = '1' then
					next_state				<= wait_data_fifo;
					next_fifo_write_en	<= '1';
				else
					next_state				<= read_data;
					next_fifo_write_en	<= '1';
					next_fifo_read_en		<= '1';
					next_write_en_sci		<= '1';
					next_data_cnt			<= data_cnt + 1;
				end if;
			end if;
			
			when emptying_fifo => 
				if fifo_empty = '1' then
					next_state				<= wait_start;
					next_busy				<= '0';
					next_data_cnt			<= 0;
				else
					next_state				<= emptying_fifo;
					next_fifo_read_en		<= '1';
				end if;
			end case;
		end process;

--num_data_to_read_int		<= to_integer(unsigned(num_data_to_read));

end Behavioral;

