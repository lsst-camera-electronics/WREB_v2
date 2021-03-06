library IEEE;
use IEEE.STD_LOGIC_1164.all;

package Base_Register_set_package_wreb is

constant schema_value			:std_logic_vector(31 downto 0)	:= x"00000001";	-- address map version
constant version_dev_level		:std_logic_vector( 3 downto 0)	:= x"b";		-- vhdl dev level (f is beta 0 is stable)
constant REB_vhdl_version		:std_logic_vector(15 downto 0)	:= x"0021";		-- REB vhdl version 
constant reserved_1_value		:std_logic_vector(31 downto 0)	:= x"00000000";	-- 
constant reserved_2_value		:std_logic_vector(31 downto 0)	:= x"00000000";	-- 
constant reserved_3_value		:std_logic_vector(31 downto 0)	:= x"00000000";	-- 

end Base_Register_set_package_wreb;

package body Base_Register_set_package_wreb is




end Base_Register_set_package_wreb;


