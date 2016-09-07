----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    17:27:06 12/11/2014 
-- Design Name: 
-- Module Name:    ad9628_dual_fast_adc_top - Behavioral 
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
use work.ad9628_dual_fast_adc_package.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

entity ad9628_dual_fast_adc_top is

port (
		clk						: in  std_logic;
		reset						: in  std_logic;
		start						: in  std_logic;
		num_data_to_read_en	: in  std_logic;
		num_data_to_read		: in  std_logic_vector(23 downto 0);
		adc_data_in_cha		: in  std_logic_vector(11 downto 0);
		adc_data_in_chb		: in  std_logic_vector(11 downto 0);
		dcoa						: in  std_logic;
		dcob						: in  std_logic;
		adc_clk_en				: out std_logic;
		busy						: out std_logic;
		adc_pdown				: out std_logic;
		write_en_sci			: out std_logic;
		adc_data_SOF			: out std_logic;
		adc_data_EOF			: out std_logic;
		num_data_to_read_rbk	: out std_logic_vector(23 downto 0);
		adc_data_out_cha		: out std_logic_vector(11 downto 0);
		adc_data_out_chb		: out std_logic_vector(11 downto 0);
		out_fast_adc_reg		: out array1724
		);

end ad9628_dual_fast_adc_top;

architecture Behavioral of ad9628_dual_fast_adc_top is

component ad9628_dual_fast_adc_fsm is
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
end component;

component generic_reg_ce_init
generic (width : integer);
port (
    reset    	: in  std_logic;           							
    clk      	: in  std_logic;           							
    ce       	: in  std_logic;  			 							
	 init	    	: in  std_logic;											
	 data_in  	: in  std_logic_vector(width downto 0);  			
	 data_out 	: out std_logic_vector(width downto 0));			
end component;

component fast_adc_fifo
  port (
    rst 		: in std_logic;
    wr_clk 	: in std_logic;
    rd_clk 	: in std_logic;
    din 		: in std_logic_vector(11 downto 0);
    wr_en 	: in std_logic;
    rd_en 	: in std_logic;
    dout 	: out std_logic_vector(11 downto 0);
    full 	: out std_logic;
    empty 	: out std_logic
  );
end component;


signal fifo_empty					: std_logic; 
signal fifo_empty_cha			: std_logic; 
signal fifo_empty_chb			: std_logic; 
signal num_data_to_read_int	: std_logic_vector(23 downto 0);
signal fifo_read_en				: std_logic;
signal fifo_write_en				: std_logic;

signal fifo_write_en_1_cha		: std_logic;
signal fifo_write_en_2_cha		: std_logic;
signal fifo_write_en_sync_cha	: std_logic;
signal fifo_write_en_1_chb		: std_logic;
signal fifo_write_en_2_chb		: std_logic;
signal fifo_write_en_sync_chb	: std_logic;

signal write_en_sci_int			: std_logic;
signal adc_data_out_cha_int	: std_logic_vector(11 downto 0);
signal adc_data_out_chb_int	: std_logic_vector(11 downto 0);


--subtype word_24 is std_logic_vector (23 downto 0);
--type array1724 is array (16 downto 0) of word_24;

signal out_fast_adc_reg_int	: array1724;

begin

	ad9628_dual_fast_adc_fsm_0 : ad9628_dual_fast_adc_fsm 
	generic map (pdown_reco_time	=> 35000)
	port map (
		clk						=> clk,
		reset						=> reset,
		start						=> start,
		fifo_empty				=> fifo_empty,
		num_data_to_read		=> num_data_to_read_int,
		busy						=> busy,
		adc_pdown				=> adc_pdown,
		adc_clk_en				=> adc_clk_en,
		fifo_read_en			=> fifo_read_en,
		fifo_write_en			=> fifo_write_en,
		write_en_sci			=> write_en_sci_int,
		adc_data_SOF			=> adc_data_SOF,
		adc_data_EOF			=> adc_data_EOF
		);

	read_data_reg :  generic_reg_ce_init 
	generic map(width => 23)
	port map (
		reset    => reset,
		clk      => clk,
		ce       => num_data_to_read_en,
		init	 	=> '0',											
		data_in  => num_data_to_read,
		data_out => num_data_to_read_int
		);

	fast_adc_fifo_cha : fast_adc_fifo
	port map (
		rst 			=> reset,
		wr_clk 		=> dcoa,
		rd_clk 		=> clk,
		din 			=> adc_data_in_cha,
		wr_en 		=> fifo_write_en_sync_cha,
		rd_en 		=> fifo_read_en,
		dout 			=> adc_data_out_cha_int,
		full 			=> open,
		empty 		=> fifo_empty_cha
		);

	fast_adc_fifo_chab: fast_adc_fifo
	port map (
		rst 			=> reset,
		wr_clk 		=> dcob,
		rd_clk 		=> clk,
		din 			=> adc_data_in_chb,
		wr_en 		=> fifo_write_en_sync_chb,
		rd_en 		=> fifo_read_en,
		dout 			=> adc_data_out_chb_int,
		full 			=> open,
		empty 		=> fifo_empty_chb
		);

	out_reg_generate: 
		for i in 1 to 16 generate 
			out_lsw_reg : generic_reg_ce_init 
			generic map(width => 23)
			port map (
				reset    => reset,           						
				clk      => clk,          						
				ce       => write_en_sci_int,  			 						
				init	 	=> '0',											
				data_in  => out_fast_adc_reg_int(i-1),  			
				data_out => out_fast_adc_reg_int(i)				
				);
			end generate;



fifo_empty	<= fifo_empty_cha or fifo_empty_chb;

out_fast_adc_reg_int(0)	<= adc_data_out_cha_int & adc_data_out_chb_int;
out_fast_adc_reg			<= out_fast_adc_reg_int;
write_en_sci				<= write_en_sci_int;
adc_data_out_cha			<= adc_data_out_cha_int;
adc_data_out_chb			<= adc_data_out_chb_int;
num_data_to_read_rbk		<= num_data_to_read_int;

flop1_write_sync_cha: FD port map (D => fifo_write_en, C => dcoa, Q => fifo_write_en_1_cha);  
flop2_write_sync_cha: FD port map (D => fifo_write_en_1_cha, C => dcoa, Q => fifo_write_en_2_cha);  
flop3_write_sync_cha: FD port map (D => fifo_write_en_2_cha, C => dcoa, Q => fifo_write_en_sync_cha);  

flop1_write_sync_chb: FD port map (D => fifo_write_en, C => dcob, Q => fifo_write_en_1_chb);  
flop2_write_sync_chb: FD port map (D => fifo_write_en_1_chb, C => dcob, Q => fifo_write_en_2_chb);  
flop3_write_sync_chb: FD port map (D => fifo_write_en_2_chb, C => dcob, Q => fifo_write_en_sync_chb); 

end Behavioral;

