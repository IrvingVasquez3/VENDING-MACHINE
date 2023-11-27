-- Dividir numero en decenas y unidades
library ieee;
use ieee.std_logic_1164.all;

entity divNum_Uni_Dec is
Port ( 
        Number : in integer range 0 to 99;
        segmentos_uni_out : out std_logic_vector (6 downto 0);
        segmentos_dec_out : out std_logic_vector (6 downto 0)
);
end divNum_Uni_Dec;

architecture Behavioral of divNum_Uni_Dec is
signal Unit : integer range 0 to 9;
signal Decena : integer range 0 to 9;

component bcd7seg_uni is
    port (
        uni: integer range 0 to 9;
        segmentos_uni : out std_logic_vector (6 downto 0)
    );
end component;

component bcd7seg_dec is
    port ( 
        dec: integer range 0 to 9;
        segmentos_dec : out std_logic_vector (6 downto 0)
    );
end component;

begin    

    obtener_unidades : bcd7seg_uni port map(
        uni => Unit,
        segmentos_uni => segmentos_uni_out
    );
    
    obtener_decenas : bcd7seg_dec port map(
        dec => Decena,
        segmentos_dec => segmentos_dec_out
    );

    process (Number)
    begin
        if Number = 0 then 
            Unit <= 0;
            Decena <= 0;
        else 
            Unit <= Number mod 10;-- Number mod 10;
            Decena <= (Number mod 100) / 10;
        end if;
    end process;
end Behavioral;
