----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 29.12.2022 10:44:34
-- Design Name: 
-- Module Name: FSM - Behavioral
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


entity FSM is
    port( clk: IN std_logic;
         s_eleccion: in std_logic;--la salido S0 de la FSM_SW
         reset: IN std_logic;
         bits_venta: out std_logic_vector(1 downto 0);
         pushbutton: IN std_logic_vector (3 downto 0)
        );
end FSM;

architecture Behavioral of FSM is
    TYPE EstadosMonedas IS (s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12,s13);
    SIGNAL present_state, next_state: EstadosMonedas;
    signal bv: integer;
begin

    process (present_state, pushbutton, clk, reset)
    begin
        if(reset='1') then
            present_state<=s0;
        elsif (rising_edge(clk)) then
            if s_eleccion='1' then
                next_state <= present_state;
                bv <= 0;
                case present_state is
                    when s0 =>
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s1;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s2;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s5;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s10;
                        end if;

                    when s1 =>
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s2;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s3;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s6;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s11;
                        end if;

                    when s2 =>
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s3;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s4;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s7;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s12;
                        end if;

                    when s3 => --Tengo 30 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s4;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s5;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s8;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;

                    when s4 => --Tengo 40 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s5;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s6;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s9;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;

                    when s5 => --Tengo 50 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s6;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s7;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s10;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;

                    when s6 => --Tengo 60 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s7;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s8;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s11;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;

                    when s7 => --Tengo 70 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s8;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s9;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s12;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;

                    when s8 => --Tengo 80 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s9;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s10;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s13;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;

                    when s9 => --Tengo 90 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s10;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s11;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s13;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;

                    when s10 => --Tengo 100 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s11;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s12;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s13;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;

                    when s11 => --Tengo 110 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s12;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s13;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s13;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;

                    when s12 => --Tengo 110 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s12;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s12;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s12;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s12;
                        end if;

                    when s13 => --Tengo 110 centimos
                        if pushbutton = "0001" then --meto 10 centimos
                            next_state <= s13;
                        elsif pushbutton = "0010" then --meto 20 centimos
                            next_state <= s13;
                        elsif pushbutton = "0100" then --meto 50 centimos
                            next_state <= s13;
                        elsif pushbutton = "1000" then --meto 1,00 euro
                            next_state <= s13;
                        end if;
                end case;
            end if;
        end if;
    end process;

    process(present_state)
    begin
        case(present_state) is
            when s12 =>
                bits_venta <= "10" after 1000ns;
                bv <= 1;
            when s13 =>
                bits_venta <= "11" after 1000ns;
            when others =>
                bits_venta <= "01";
                bv <= 0;
        end case;
    end process;
    next_state <= s0 when bv = 1;
end Behavioral;


