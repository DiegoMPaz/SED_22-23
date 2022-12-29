----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 29.12.2022 10:45:22
-- Design Name: 
-- Module Name: FSM_SW - Behavioral
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
use ieee.std_logic_arith.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;


entity FSM_SW is
    port(
        clk: in std_logic;
        reset:in std_logic;
        sw1: in std_logic;
        sw2: in std_logic;
        sw3: in std_logic;
        sw4: in std_logic;
        led: out std_logic_vector(3 downto 0);
        s_eleccion:out std_logic :='0'
    );
end FSM_SW;

architecture Behavioral of FSM_SW is
    TYPE REFRESCOS IS (nulo, agua, fanta_naranja, pepsi, aquarius);
    SIGNAL present_state, next_state: REFRESCOS;
    signal switches : std_logic_vector(3 downto 0):="0000";

begin
    process (present_state)
    begin
        switches<=sw1&sw2&sw3&sw4;
        next_state <= present_state;
        if(switches="1000") then
            present_state<=agua;
            s_eleccion<='1';
            led(0)<='1';
        elsif (switches="0100") then
            present_state<=fanta_naranja;
            s_eleccion<='1';
            led(1)<='1';
        elsif (switches="0010") then
            present_state<=pepsi;
            s_eleccion<='1';
            led(2)<='1';
        elsif(switches="0001") then
            present_state<=aquarius;
            s_eleccion<='1';
            led(3)<='1';
        else
            present_state<=nulo;
            led(0)<='0';
            led(1)<='0';
            led(2)<='0';
            led(3)<='0';
            s_eleccion<='0';
        end if;
    end process;

    process (clk, reset)
    begin
        if(reset='1') then
            present_state<=nulo;
        elsif (rising_edge(clk)) then
            present_state<=next_state;
        end if;
    end process;

end Behavioral;