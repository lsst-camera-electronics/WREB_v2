----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    17:59:52 12/04/2014 
-- Design Name: 
-- Module Name:    adt_7420_and_ltc2489_fsm - Behavioral 
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

entity adt_7420_and_ltc2489_fsm is
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

end adt_7420_and_ltc2489_fsm;

architecture Behavioral of adt_7420_and_ltc2489_fsm is

type   state_type is (wait_start, adt_write_reg,
							 adt_wait_write, adt_wait_rd_1,
							 adt_rd_1, adt_wait_rd_2, 
							 adt_rd_2, adt_incr_add,
							 ltc_wait_write, ltc_write_reg, 
							 ltc_wait_conv, 
							 ltc_wait_rd_1, ltc_rd_1, 
							 ltc_wait_rd_2, ltc_rd_2,
							 ltc_wait_rd_3, ltc_rd_3
							 );

	signal pres_state, next_state : state_type;
	signal next_busy					: std_logic;
	signal next_start_i2c			: std_logic;
	signal next_i2c_rw				: std_logic;
	signal next_device_addr			: STD_LOGIC_VECTOR(6 DOWNTO 0); 
	signal next_data_wr				: STD_LOGIC_VECTOR(7 DOWNTO 0); 
	signal next_adt_add_cnt_int	: std_logic_vector(1 downto 0);
	signal next_en_adt_msw_int		: std_logic;
	signal next_en_adt_lsw_int		: std_logic;
	signal next_ltc_conv_cnt_int	: std_logic_vector(23 downto 0);
	signal next_latch_en_ltc		: std_logic_vector(2 downto 0);
	

	signal adt_add_cnt_int			: std_logic_vector(1 downto 0);
	signal en_adt_msw_int			: std_logic;
	signal en_adt_lsw_int			: std_logic;
	signal ltc_conv_cnt_int			: std_logic_vector(23 downto 0);
	
	

begin

process (clk)
  begin
    if clk'event and clk = '1' then
      if reset = '1' then
			pres_state 			<= wait_start;
			busy					<= '0';
			start_i2c			<= '0';
			i2c_rw				<= '0';
			device_addr			<= (others => '0');
			data_wr				<= (others => '0');
			adt_add_cnt_int	<= (others => '0');
			en_adt_msw_int		<= '0';
			en_adt_lsw_int		<= '0';
			ltc_conv_cnt_int	<= (others => '0');
			latch_en_ltc		<= (others => '0');
			
		else
			pres_state 			<= next_state;
			busy					<= next_busy;
			start_i2c			<= next_start_i2c;
			i2c_rw				<= next_i2c_rw;
			device_addr			<= next_device_addr;
			data_wr				<= next_data_wr;
			adt_add_cnt_int	<= next_adt_add_cnt_int;
			en_adt_msw_int		<= next_en_adt_msw_int;
			en_adt_lsw_int		<= next_en_adt_lsw_int;
			ltc_conv_cnt_int	<= next_ltc_conv_cnt_int;
			latch_en_ltc		<= next_latch_en_ltc;

			
			
		end if;
    end if;
  end process;


process (start_read_board_t, start_read_chip_t, busy_i2c, adt_add_cnt_int, ltc_conv_cnt_int, read_chip_add, pres_state)
 
  begin

    -------------------- outputs default values --------------------
			next_busy					<= '1';
			next_start_i2c				<= '0';
			next_i2c_rw					<= '0';
			next_device_addr			<= (others => '0');
			next_data_wr				<= (others => '0');
			next_adt_add_cnt_int		<= adt_add_cnt_int;
			next_en_adt_msw_int		<= '0';
			next_en_adt_lsw_int		<= '0';
			next_ltc_conv_cnt_int	<= ltc_conv_cnt_int;
			next_latch_en_ltc			<= (others => '0');

	
	case pres_state is
    
		when wait_start =>
			if start_read_board_t = '1' and start_read_chip_t = '0' then
				next_state			<= adt_wait_write;
				next_i2c_rw			<= '0';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
								
			elsif start_read_board_t = '0' and start_read_chip_t = '1' then
				next_state			<= ltc_wait_write;
				next_i2c_rw			<= '0';
				next_device_addr	<= "0010100";
				next_data_wr		<= "1011" & read_chip_add(1) & "00" & read_chip_add(0);
				next_start_i2c		<= '1';			
			else
				next_state 					<= wait_start;
				next_busy					<= '0';
				next_adt_add_cnt_int		<= (others => '0');
				next_ltc_conv_cnt_int	<= (others => '0');
			end if;
			
		when adt_wait_write => 
			if busy_i2c = '0' then
				next_state			<= adt_wait_write;
				next_i2c_rw			<= '0';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
			else
				next_state			<= adt_write_reg;
				next_i2c_rw			<= '1';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
			end if;
		
		when adt_write_reg => 
			if busy_i2c = '1' then
				next_state			<= adt_write_reg;
				next_i2c_rw			<= '1';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
				
			else 
				next_state			<= adt_wait_rd_1;
				next_i2c_rw			<= '1';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
			end if;
			
		when adt_wait_rd_1 => 
			if busy_i2c = '0' then
				next_state			<= adt_wait_rd_1;
				next_i2c_rw			<= '1';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
			else
				next_state			<= adt_rd_1;
				next_i2c_rw			<= '1';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
			end if;
			
		when adt_rd_1 => 
			if busy_i2c = '1' then
				next_state			<= adt_rd_1;
				next_i2c_rw			<= '1';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
			else
				next_state			<= adt_wait_rd_2;
				next_i2c_rw			<= '1';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
				next_en_adt_msw_int	<= '1';
			end if;
			
		when adt_wait_rd_2 => 
			if busy_i2c = '0' then
				next_state			<= adt_wait_rd_2; 
				next_i2c_rw			<= '1';
				next_device_addr	<= "10010" & adt_add_cnt_int;
				next_data_wr		<= x"00";
				next_start_i2c		<= '1';
				next_en_adt_msw_int	<= '1';
			else
				next_state			<= adt_rd_2;
			end if;
			
		when adt_rd_2 => 
			if busy_i2c = '1' then
				next_state			<= adt_rd_2;
			else
				if adt_add_cnt_int = "11" then
					next_state				<= wait_start;
					next_en_adt_lsw_int		<= '1';
				else
					next_state				<= adt_incr_add;
					next_adt_add_cnt_int	<= adt_add_cnt_int + 1;
					next_en_adt_lsw_int		<= '1';
				end if;
			end if;
		
		when adt_incr_add => 
			next_state			<= adt_wait_write;
			next_i2c_rw			<= '0';
			next_device_addr	<= "10010" & adt_add_cnt_int;
			next_data_wr		<= x"00";
			next_start_i2c		<= '1';

-- read ltc

		when ltc_wait_write => 
			if busy_i2c = '0' then
				next_state			<= ltc_wait_write;
				next_i2c_rw			<= '0';
				next_device_addr	<= "0010100";
				next_data_wr		<= "1011" & read_chip_add(1) & "00" & read_chip_add(0);
				next_start_i2c		<= '1';	
			else
				next_state			<= ltc_write_reg;
			end if;
			
		when ltc_write_reg => 
			if busy_i2c = '1' then
				next_state			<= ltc_write_reg;
			else 
				next_state					<= ltc_wait_conv;
				next_ltc_conv_cnt_int	<= ltc_conv_cnt_int + 1;
			end if;
			
		when ltc_wait_conv => 
			if ltc_conv_cnt_int = x"E4E1C0" then
				next_state			<= ltc_wait_rd_1;
				next_i2c_rw			<= '1';
				next_device_addr	<= "0010100";
				next_start_i2c		<= '1';	
			else 
				next_state					<= ltc_wait_conv;
				next_ltc_conv_cnt_int	<= ltc_conv_cnt_int + 1;
			end if;
		
		when ltc_wait_rd_1 => 
			if busy_i2c = '0' then
				next_state			<= ltc_wait_rd_1;
				next_i2c_rw			<= '1';
				next_device_addr	<= "0010100";
				next_start_i2c		<= '1';	
			else 
				next_state			<= ltc_rd_1;
				next_i2c_rw			<= '1';
				next_device_addr	<= "0010100";
				next_start_i2c		<= '1';	
			end if;
			
		when ltc_rd_1 => 
			if busy_i2c = '1' then
				next_state			<= ltc_rd_1;
				next_i2c_rw			<= '1';
				next_device_addr	<= "0010100";
				next_start_i2c		<= '1';	
			else 
				next_state					<= ltc_wait_rd_2;
				next_i2c_rw					<= '1';
				next_device_addr			<= "0010100";
				next_start_i2c				<= '1';	
				next_latch_en_ltc			<= "100";
			end if;
			
		when ltc_wait_rd_2 => 
			if busy_i2c = '0' then
				next_state			<= ltc_wait_rd_2;
				next_i2c_rw			<= '1';
				next_device_addr	<= "0010100";
				next_start_i2c		<= '1';	
			else 
				next_state			<= ltc_rd_2;
				next_i2c_rw			<= '1';
				next_device_addr	<= "0010100";
				next_start_i2c		<= '1';	
			end if;
			
		when ltc_rd_2 => 
			if busy_i2c = '1' then
				next_state			<= ltc_rd_2;
				next_i2c_rw			<= '1';
				next_device_addr	<= "0010100";
				next_start_i2c		<= '1';	
			else 
				next_state					<= ltc_wait_rd_3;
				next_i2c_rw					<= '1';
				next_device_addr			<= "0010100";
				next_start_i2c				<= '1';	
				next_latch_en_ltc			<= "010";
			end if;
			
		when ltc_wait_rd_3 => 
			if busy_i2c = '0' then
				next_state			<= ltc_wait_rd_3;
				next_i2c_rw			<= '1';
				next_device_addr	<= "0010100";
				next_start_i2c		<= '1';	
			else 
				next_state			<= ltc_rd_3;
			end if;
			
		when ltc_rd_3 => 
			if busy_i2c = '1' then
				next_state			<= ltc_rd_3;
			else 
				next_state					<= wait_start;
				next_latch_en_ltc			<= "001";
			end if;
		end case;
	end process;





	
process (clk)
begin

-- default state

if clk'event and clk = '1' then  -- rising clock edge
    if reset = '1' then                  -- synchronous reset 
      latch_en_bus_msw_adt <= (others => '0');
    else
		case adt_add_cnt_int is
			when "00" => latch_en_bus_msw_adt(0) <= en_adt_msw_int;
			when "01" => latch_en_bus_msw_adt(1) <= en_adt_msw_int;
			when "10" => latch_en_bus_msw_adt(2) <= en_adt_msw_int;
			when "11" => latch_en_bus_msw_adt(3) <= en_adt_msw_int;
		
			when others => latch_en_bus_msw_adt <= (others => '0');
		end case;
	end if;
 end if;
end process;
				

				
process (clk)
begin

-- default state

if clk'event and clk = '1' then  -- rising clock edge
    if reset = '1' then                  -- synchronous reset 
      latch_en_bus_lsw_adt <= (others => '0');
    else
   case adt_add_cnt_int is
      when "00" => latch_en_bus_lsw_adt(0) <= en_adt_lsw_int;
      when "01" => latch_en_bus_lsw_adt(1) <= en_adt_lsw_int;
      when "10" => latch_en_bus_lsw_adt(2) <= en_adt_lsw_int;
      when "11" => latch_en_bus_lsw_adt(3) <= en_adt_lsw_int;
		
      when others => latch_en_bus_lsw_adt <= (others => '0');
   end case;
 end if;
 end if;
end process;				
				
					

end Behavioral;

