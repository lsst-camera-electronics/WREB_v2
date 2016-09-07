--
--	Package File Template
--
--	Purpose: This package defines supplemental types, subtypes, 
--		 constants, and functions 
--
--   To use any of the example code shown below, uncomment the lines and modify as necessary
--

library IEEE;
use IEEE.STD_LOGIC_1164.all;

package ltc2945_add_package_wreb is


constant V_HTR_dev_add		: std_logic_vector (6 downto 0) := x"D"&"001";		-- D2 ??V
constant V_DREB_dev_add		: std_logic_vector (6 downto 0) := x"D"&"010";		-- D4 3.3V
constant V_CLK_H_dev_add	: std_logic_vector (6 downto 0) := x"D"&"011";		-- D6 ??V
constant V_DPHI_dev_add		: std_logic_vector (6 downto 0) := x"D"&"100";		-- D8 ??V
constant V_ANA_dev_add		: std_logic_vector (6 downto 0) := x"D"&"110";		-- DC 5V
constant V_OD_dev_add		: std_logic_vector (6 downto 0) := x"D"&"111";		-- DE 40V

constant V_reg_add			: std_logic_vector (7 downto 0) := x"1E";
constant I_reg_add			: std_logic_vector (7 downto 0) := x"14";


end ltc2945_add_package_wreb;

package body ltc2945_add_package_wreb is

end ltc2945_add_package_wreb;
