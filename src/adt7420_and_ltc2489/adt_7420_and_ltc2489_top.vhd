----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    18:28:56 12/05/2014 
-- Design Name: 
-- Module Name:    adt_7420_and_ltc2489_top - Behavioral 
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
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity adt_7420_and_ltc2489_top is

port (
		clk 						: in  std_logic;
		reset						: in  std_logic;
		start_read_board_t	: in  std_logic;
		start_read_chip_t		: in  std_logic;
		read_chip_add			: in  std_logic_vector(1 downto 0);
		
		busy						: out std_logic;
		error_board_T1			: out std_logic;
		board_T1_out			: out std_logic_vector(15 downto 0);
		error_board_T2			: out std_logic;
		board_T2_out			: out std_logic_vector(15 downto 0);
		error_board_T3			: out std_logic;
		board_T3_out			: out std_logic_vector(15 downto 0);
		error_board_T4			: out std_logic;
		board_T4_out			: out std_logic_vector(15 downto 0);
		
		error_chip_t			: out std_logic;
		chip_t					: out std_logic_vector(23 downto 0);
		
		sda       				: INOUT  STD_LOGIC;                    --serial data output of i2c bus
		scl       				: INOUT  STD_LOGIC		               --serial clock output of i2c bus
		);

end adt_7420_and_ltc2489_top;

architecture Behavioral of adt_7420_and_ltc2489_top is

component adt_7420_and_ltc2489_fsm is
port (
		clk 						: in  std_logic;
		reset						: in  std_logic;
		start_read_board_t	: in  std_logic;
		start_read_chip_t		: in  std_logic;
		read_chip_add			: in  std_logic_vector(1 downto 0);
		busy_i2c					: in  std_logic;
		busy						: out std_logic;
		start_i2c				: out std_logic;
		i2c_rw					: out std_logic;
		device_addr				: out STD_LOGIC_VECTOR(6 DOWNTO 0); --address of target slave
		data_wr					: out STD_LOGIC_VECTOR(7 DOWNTO 0); --address of target slave
		latch_en_bus_msw_adt	: out std_logic_vector(3 downto 0);
		latch_en_bus_lsw_adt	: out std_logic_vector(3 downto 0);
		latch_en_ltc			: out std_logic_vector(2 downto 0)
		);
end component;

component i2c_master IS
  GENERIC(
    input_clk : INTEGER := 100_000_000; --input clock speed from user logic in Hz
    bus_clk   : INTEGER := 400_000);   --speed the i2c bus (scl) will run at in Hz
  PORT(
    clk       : IN     STD_LOGIC;                    --system clock
    reset     : IN     STD_LOGIC;                    --active low reset
    ena       : IN     STD_LOGIC;                    --latch in command
    addr      : IN     STD_LOGIC_VECTOR(6 DOWNTO 0); --address of target slave
    rw        : IN     STD_LOGIC;                    --'0' is write, '1' is read
    data_wr   : IN     STD_LOGIC_VECTOR(7 DOWNTO 0); --data to write to slave
    busy      : OUT    STD_LOGIC;                    --indicates transaction in progress
    data_rd   : OUT    STD_LOGIC_VECTOR(7 DOWNTO 0); --data read from slave
    ack_error : out STD_LOGIC;                    --flag if improper acknowledge from slave
    sda       : INOUT  STD_LOGIC;                    --serial data output of i2c bus
    scl       : INOUT  STD_LOGIC);                   --serial clock output of i2c bus
END component;

component generic_reg_ce_init is
generic ( width : integer := 7 );
port (
    reset    : in  std_logic;           							-- syncronus reset
    clk      : in  std_logic;           							-- clock
    ce       : in  std_logic;  			 							-- clock enable
	 init	    : in  std_logic;											-- signal to reset the reg (active high)
	 data_in  : in  std_logic_vector(width downto 0);  			-- data in
	 data_out : out std_logic_vector(width downto 0));				-- data out
end component;

component ff_ce is
port (
    reset    : in  std_logic;           -- syncronus reset
    clk      : in  std_logic;           -- clock
    data_in  : in  std_logic;  			 -- data in
	 ce       : in  std_logic;  			 -- clock enable
    data_out : out std_logic);          -- data out
end component;

signal busy_i2c					: std_logic;
signal start_i2c					: std_logic;
signal i2c_rw						: std_logic;
signal device_addr				: STD_LOGIC_VECTOR(6 DOWNTO 0); --address of target slave
signal data_to_12c				: STD_LOGIC_VECTOR(7 DOWNTO 0); --address of target slave
signal latch_en_bus_msw_adt	: std_logic_vector(3 downto 0);
signal latch_en_bus_lsw_adt	: std_logic_vector(3 downto 0);
signal i2c_data_rd 				: std_logic_vector(7 DOWNTO 0); --data read from slave
signal i2c_ack_error				: std_logic;
signal adt_error_bus				: std_logic_vector(3 downto 0);

signal latch_en_ltc				: std_logic_vector(2 downto 0);

signal read_chip_add_int : std_logic_vector(1 downto 0);



subtype word_8 is std_logic_vector (7 downto 0);
type array48 is array (3 downto 0) of word_8;
type array38 is array (2 downto 0) of word_8;

signal adt_out_lsw_array		: array48;
signal adt_out_MSW_array		: array48;

signal ltc_out_array			: array38;

begin

adt_7420_and_ltc2489_fsm_0 : adt_7420_and_ltc2489_fsm 
port map (
		clk 						=> clk,
		reset						=> reset,
		start_read_board_t	=> start_read_board_t,
		start_read_chip_t		=> start_read_chip_t,
		read_chip_add			=> read_chip_add_int,
		busy_i2c					=> busy_i2c,
		
		busy						=> busy,
		start_i2c				=> start_i2c,
		i2c_rw					=> i2c_rw,
		device_addr				=> device_addr,
		data_wr					=> data_to_12c,
		latch_en_bus_msw_adt	=> latch_en_bus_msw_adt,
		latch_en_bus_lsw_adt	=> latch_en_bus_lsw_adt,
		latch_en_ltc			=> latch_en_ltc
		);

i2c_master_0 : i2c_master 
  generic map(
    input_clk => 100_000_000, --input clock speed from user logic in Hz
    bus_clk   =>     400_000)   --speed the i2c bus (scl) will run at in Hz
  port map(
    clk       		=> clk,                    --system clock
    reset   		=> reset,                   --active low reset
    ena       		=> start_i2c,                    --latch in command
    addr      		=> device_addr,								 --address of target slave
    rw        		=> i2c_rw,                    --'0' is write, '1' is read
    data_wr   		=> data_to_12c, 					--data to write to slave
    busy      		=> busy_i2c,                   --indicates transaction in progress
    data_rd   		=> i2c_data_rd, 					--data read from slave
    ack_error 		=> i2c_ack_error,                    --flag if improper acknowledge from slave
    sda       		=> sda,                   --serial data output of i2c bus
    scl       		=> scl
	 );                  

adt_lsw_reg_generate: 
			for i in 0 to 3 generate 
			adt_out_lsw_reg : generic_reg_ce_init 
			generic map(width => 7)
			port map (
				reset    => reset,           						
				clk      => clk,          						
				ce       => latch_en_bus_lsw_adt(i),  			 						
				init	 	=> '0',											
				data_in  => i2c_data_rd,  			
				data_out => adt_out_lsw_array(I)				
				);
			end generate;

adt_MSW_reg_generate: 
			for i in 0 to 3 generate 
			adt_out_MSW_reg : generic_reg_ce_init 
			generic map(width => 7)
			port map (
				reset    => reset,           						
				clk      => clk,          						
				ce       => latch_en_bus_MSW_adt(i),  			 						
				init	 	=> '0',											
				data_in  => i2c_data_rd,  			
				data_out => adt_out_MSW_array(I)				
				);
			end generate;

error_ff_generate: 
	for i in 0 to 3 generate 
error_adt_ff : ff_ce 
port map (
    reset    => reset,        
    clk      => clk,              
	 data_in  => i2c_ack_error, 	 
	 ce       => latch_en_bus_lsw_adt(i),
    data_out => adt_error_bus(i)); 
	 end generate;

error_ltc_ff : ff_ce 
port map (
    reset    => reset,        
    clk      => clk,              
	 data_in  => i2c_ack_error, 	 
	 ce       => latch_en_ltc(0), 
    data_out => error_chip_t); 

chip_t_reg_generate: 
			for i in 0 to 2 generate 
			chip_t_reg : generic_reg_ce_init 
			generic map(width => 7)
			port map (
				reset    => reset,           						
				clk      => clk,          						
				ce       => latch_en_ltc(i),  			 						
				init	 	=> '0',											
				data_in  => i2c_data_rd,  			
				data_out => ltc_out_array(I)				
				);
			end generate;

	chip_add_reg : generic_reg_ce_init 
			generic map(width => 1)
			port map (
				reset    => reset,           						
				clk      => clk,          						
				ce       => start_read_chip_t,  			 						
				init	 	=> '0',											
				data_in  => read_chip_add,  			
				data_out => read_chip_add_int				
				);


board_T1_out	<= adt_out_MSW_array(0) & adt_out_lsw_array(0);	
board_T2_out	<= adt_out_MSW_array(1) & adt_out_lsw_array(1);	
board_T3_out	<= adt_out_MSW_array(2) & adt_out_lsw_array(2);	
board_T4_out	<= adt_out_MSW_array(3) & adt_out_lsw_array(3);	

error_board_T1	<= adt_error_bus(0);
error_board_T2	<= adt_error_bus(1);
error_board_T3	<= adt_error_bus(2);
error_board_T4	<= adt_error_bus(3);

chip_t			<= ltc_out_array(2) & ltc_out_array(1) & ltc_out_array(0);


end Behavioral;

