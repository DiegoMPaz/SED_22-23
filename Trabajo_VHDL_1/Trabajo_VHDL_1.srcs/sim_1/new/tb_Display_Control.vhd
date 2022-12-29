----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 29.12.2022 11:07:03
-- Design Name: 
-- Module Name: tb_Display_Control - Behavioral
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

entity tb_Display_Control is
end tb_Display_Control;

architecture tb of tb_Display_Control is

    component Display_Control
        port (num       : in std_logic_vector (3 downto 0);
              clk       : in std_logic;
              Venerr    : in std_logic_vector (1 downto 0);
              digsel    : out std_logic_vector (7 downto 0);
              segmentos : out std_logic_vector (7 downto 0));
    end component;

    signal num       : std_logic_vector (3 downto 0);
    signal clk       : std_logic;
    signal Venerr    : std_logic_vector (1 downto 0);
    signal digsel    : std_logic_vector (7 downto 0);
    signal segmentos : std_logic_vector (7 downto 0);

    constant TbPeriod : time := 10 ns; -- EDIT Put right period here
    signal TbClock : std_logic := '0';
    signal TbSimEnded : std_logic := '0';

begin

    dut : Display_Control
    port map (num       => num,
              clk       => clk,
              Venerr    => Venerr,
              digsel    => digsel,
              segmentos => segmentos);

    TbClock <= not TbClock after TbPeriod/2 when TbSimEnded /= '1' else '0';

    clk <= TbClock;

    stimuli : process
    begin
    
    for i in 0 to num'HIGH LOOP
        num <= (others => '0');
        Venerr <= (others => '0');

             wait for 5ns;
        num<="0011";
        Venerr <= "00";
        
             wait for 12ms;
       num<="0000";
       Venerr <= "01";
       
             wait for 10ms;
       num<="0000";
       Venerr <= "00";

             wait for 10ms;
       num<="0000";
       Venerr <= "00";

        TbSimEnded <= '1';
        wait;
        END LOOP;
    end process;

end tb;

