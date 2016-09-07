----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    01:22:52 02/18/2015 
-- Design Name: 
-- Module Name:    clock_divider - Behavioral 
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
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity clock_divider is

generic ( clock_in_frequency : integer := 100000000;
			 clock_out_frequency: integer := 2);
port (
	clk_in 	: in  std_logic;
	clk_out  : out std_logic);

end clock_divider;

architecture Behavioral of clock_divider is
	
--	 constant divider  :  integer := (clock_in_frequency/clock_out_frequency)/2; 
--    -- prescaler should be (clock_speed/desired_clock_speed)/2 because you want a rising edge every period
----    signal prescaler: STD_LOGIC_VECTOR(23 downto 0) := "101111101011110000100000"; -- 12,500,000 in binary
--    signal prescaler_counter: STD_LOGIC_VECTOR(31 downto 0) := (others => '0');
--    signal newClock : std_logic := '0';
--begin
--
--    countClock: process(clk_in, newClock)
--    begin
--        if rising_edge(clk_in) then
--            prescaler_counter <= prescaler_counter + 1;
--            if(prescaler_counter > x"fffffff9") then
--                -- Iterate
--                newClock <= not newClock;
--
--                prescaler_counter <= (others => '0');
--            end if;
--        end if;
--    end process;
--
--clk_out	<= newClock;
--
--end Behavioral;




signal count_0 : std_logic_vector(31 downto 0); 
signal count_1 : std_logic_vector(31 downto 0);
signal cnt		: std_logic_vector(31 downto 0);
 
signal led_output : std_logic;

begin

PWM_MODULE_0: process(clk_in) begin

	if rising_edge(clk_in)then 
		count_0 <= count_0 + 1;
		if count_0 < count_1 then
			led_output <= '1'; 
		else
			led_output <= '0'; 
		end if;
	end if;
end process PWM_MODULE_0;

COUNTER_0: process(clk_in) 
begin

	if rising_edge(clk_in) then 
		if cnt = x"000000fe" then
			count_1 <= count_1 + 1;
			cnt <= cnt + 1; 
		elsif cnt = x"000000ff" then
			count_1 <= count_1 - 1; 
		else
			cnt <= cnt + 1; 
	end if;
end if;
end process COUNTER_0;
clk_out <= led_output; 
end Behavioral;
