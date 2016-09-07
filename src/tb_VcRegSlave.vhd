--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   18:11:58 02/19/2015
-- Design Name:   
-- Module Name:   C:/Users/srusso/Desktop/DREB_v2/src/tb_VcRegSlave.vhd
-- Project Name:  DREB_v2
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: VcRegSlave
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
 
ENTITY tb_VcRegSlave IS
END tb_VcRegSlave;
 
ARCHITECTURE behavior OF tb_VcRegSlave IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT VcRegSlave
    PORT(
         vcRxOut : IN  std_logic;
         vcRxCommonOut : IN  std_logic;
         vcTxIn : OUT  std_logic;
         vcTxOut : IN  std_logic;
         regSlaveIn : IN  std_logic;
         regSlaveOut : OUT  std_logic;
         locClk : IN  std_logic;
         locRst : IN  std_logic;
         vcTxClk : IN  std_logic;
         vcTxRst : IN  std_logic;
         vcRxClk : IN  std_logic;
         vcRxRst : IN  std_logic;
         CScopeControl : INOUT  std_logic_vector(35 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal vcRxOut : std_logic := '0';
   signal vcRxCommonOut : std_logic := '0';
   signal vcTxOut : std_logic := '0';
   signal regSlaveIn : std_logic := '0';
   signal locClk : std_logic := '0';
   signal locRst : std_logic := '0';
   signal vcTxClk : std_logic := '0';
   signal vcTxRst : std_logic := '0';
   signal vcRxClk : std_logic := '0';
   signal vcRxRst : std_logic := '1';

	--BiDirs
   signal CScopeControl : std_logic_vector(35 downto 0);

 	--Outputs
   signal vcTxIn : std_logic;
   signal regSlaveOut : std_logic;

   -- Clock period definitions
   constant locClk_period : time := 10 ns;
   constant vcTxClk_period : time := 10 ns;
   constant vcRxClk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: VcRegSlave PORT MAP (
          vcRxOut => vcRxOut,
          vcRxCommonOut => vcRxCommonOut,
          vcTxIn => vcTxIn,
          vcTxOut => vcTxOut,
          regSlaveIn => regSlaveIn,
          regSlaveOut => regSlaveOut,
          locClk => locClk,
          locRst => locRst,
          vcTxClk => vcTxClk,
          vcTxRst => vcTxRst,
          vcRxClk => vcRxClk,
          vcRxRst => vcRxRst,
          CScopeControl => CScopeControl
        );

   -- Clock process definitions
   locClk_process :process
   begin
		locClk <= '0';
		wait for locClk_period/2;
		locClk <= '1';
		wait for locClk_period/2;
   end process;
 
   vcTxClk_process :process
   begin
		vcTxClk <= '0';
		wait for vcTxClk_period/2;
		vcTxClk <= '1';
		wait for vcTxClk_period/2;
   end process;
 
   vcRxClk_process :process
   begin
		vcRxClk <= '0';
		wait for vcRxClk_period/2;
		vcRxClk <= '1';
		wait for vcRxClk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
      wait for 100 ns;	

      wait for locClk_period*10;

		vcRxRst	<= '0';
      -- insert stimulus here 

      wait;
   end process;

END;
