----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 29.12.2022 11:08:12
-- Design Name: 
-- Module Name: tb_FSM - Behavioral
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

entity tb_FSM is
--  Port ( );
end tb_FSM;

architecture tb of tb_FSM is

    component FSM
        port (clk        : in std_logic;
              s_eleccion : in std_logic;
              reset      : in std_logic;
              bits_venta : out std_logic_vector (1 downto 0);
              pushbutton : in std_logic_vector (3 downto 0));
    end component;

    signal clk        : std_logic;
    signal s_eleccion : std_logic;
    signal reset      : std_logic;
    signal bits_venta : std_logic_vector (1 downto 0);
    signal pushbutton : std_logic_vector (3 downto 0);

    constant TbPeriod : time := 10 ns; -- EDIT Put right period here
    signal TbClock : std_logic := '0';
    signal TbSimEnded : std_logic := '0';

begin

    dut : FSM
    port map (clk        => clk,
              s_eleccion => s_eleccion,
              reset      => reset,
              bits_venta => bits_venta,
              pushbutton => pushbutton);

    -- Clock generation
    TbClock <= not TbClock after TbPeriod/2 when TbSimEnded /= '1' else '0';

    -- EDIT: Check that clk is really your main clock signal
    clk <= TbClock;

    stimuli : process
    begin
        -- EDIT Adapt initialization as needed
        s_eleccion <= '0','1' after 100ns,'0' after 1000ns,'1' after 1200ns;
        pushbutton<="0000","0010" after 200ns,"1010" after 500ns,"1101" after 1200ns;
        reset<='1','0' after 2000ns,'1' after 3000ns;


        -- Reset generation
        -- EDIT: Check that reset is really your reset signal
        --reset <= '1';
        --wait for 10000 ns;
        --reset <= '0';
        --wait for 10000 ns;

        -- EDIT Add stimuli here
        --wait for 100 * TbPeriod;

        -- Stop the clock and hence terminate the simulation
        --TbSimEnded <= '1';
        wait;
    end process;

end tb;
