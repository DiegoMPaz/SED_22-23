----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 29.11.2022 15:30:28
-- Design Name: 
-- Module Name: TOP - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
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
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity TOP is
--  Port ( );
end TOP;

architecture Behavioral of TOP is

component synchrnzer is
        port(
            clk: in std_logic;
            async_in: in std_logic;
            sync_out:out std_logic;
            reset: in std_logic
        );
    end component;
    
begin
 inst_sincronizador1: synchrnzer port map(
           clk=>'1',
            async_in=>'1',
            sync_out=>'1',
            reset=>'0'
        );

end Behavioral;
