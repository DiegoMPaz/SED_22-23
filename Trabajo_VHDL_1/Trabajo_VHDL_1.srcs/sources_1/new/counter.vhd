----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 29.12.2022 10:44:14
-- Design Name: 
-- Module Name: counter - Behavioral
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




entity counter is
    Port (
        clk: in std_logic;
        c10,c20,c50,e1,reset:in std_logic;
        num: out std_logic_vector (3 downto 0);
        s_eleccion:in std_logic;
        bits_venta:in std_logic_vector(1 downto 0)
    );
end counter;

architecture Behavioral of counter is
    signal suma : UNSIGNED(4 downto 0):="00000";--5 bits para el carry-out

begin

    process(clk,reset)
    begin
        if(s_eleccion='1')  then
            if reset='0' or bits_venta = "10" or bits_venta = "11" then 
                suma<="00000";
            elsif rising_edge(CLK) then
                if c10='1' then
                    suma<=suma+"00001";
                end if;
                if c20='1' then
                    suma<=suma+"00010";
                end if;
                if c50='1' then
                    suma<=suma+"00101";
                end if;
                if e1='1' then
                    suma<=suma+"01010";
                end if;
            end if;
        else
            suma<="00000";
        end if;
    end process;

    process (suma)
    begin
        case suma is
            when "00000"=>num<="0000";-- 0 cent
            when "00001"=>num<="0001";--10 cent
            when "00010"=>num<="0010";--20 cent
            when "00011"=>num<="0011";--30 cent
            when "00100"=>num<="0100";--40 cent
            when "00101"=>num<="0101";--50 cent
            when "00110"=>num<="0110";--60 cent
            when "00111"=>num<="0111";--70 cent
            when "01000"=>num<="1000";--80 cent
            when "01001"=>num<="1001";--90 cent
            when "01010"=>num<="1010";--1.00 euros
            when "01011"=>num<="1011";--1.10 euros
            when "01100"=>num<="1100";--1.20 euros
            when others=>num<="1101";--error
        end case;
    end process;

end Behavioral;
