-- Demultiplexor para mostrar numeros en display
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity demux_display is
  Port (
  -- contador 1000 Hz. conteoMax= 100000. T=0.001=1ms . Cada 2 ms  cambia 
  -- conteoMax= 100Mhz / (freq req) = 100Mhz / (1/segundosReq)
        clk1000hz: 	    in std_logic; 
        Dispensar_IN :  in std_logic; 
        Dar_cambio_IN : in std_logic; 
        Confirmar_compra_IN : std_logic; 
        producto_selected : in std_logic; 
        segmentos_uni_dinero : in std_logic_vector (6 downto 0);
        segmentos_dec_dinero : in std_logic_vector (6 downto 0);
        segmentos_uni_cambio : in std_logic_vector (6 downto 0);
        segmentos_dec_cambio : in std_logic_vector (6 downto 0);
        segmentos_uni_precio : in std_logic_vector (6 downto 0);
        segmentos_dec_precio : in std_logic_vector (6 downto 0);
        segmentos_letra : in std_logic_vector (6 downto 0);
        display: 	    out std_logic_vector(6 downto 0);
        curr_display:   out std_logic_vector(3 downto 0)
  
  );
end demux_display;

architecture Behavioral of demux_display is
    signal refresh_state: std_logic_vector(1 downto 0); --:= ('1','1');--
    signal display_sel: std_logic_vector(3 downto 0); -- := (others => '0'); --selector de 4 bits

begin
    gen_clock: process(clk1000hz)
    begin
        if (clk1000hz'event and clk1000hz='1') then  
            refresh_state <= refresh_state + 1;
        end if; 
    end process; 
    
    

    show_display: process(refresh_state) 
    begin 
        -- seleccion del display 
        -- Con 0-ON y con 1-OFF
        case refresh_state is 
            when "00" => 
                display_sel <= "1110"; -- display 0 
            when "01" => 
                display_sel <= "1101"; -- display 1 
            when "10" => 
                display_sel <= "1011"; -- display 2 
            when "11" => 
                display_sel <= "0111"; -- display 3 
            when others => 
                display_sel <= "1111"; 
        end case; 
        curr_display <= display_sel;

        -- mostrar digitos 
        case display_sel is 
            when "1110" => -- DISPLAY 1 ACTIVO 
		        if (Confirmar_compra_IN = '1')then -- Dar_cambio_IN por dispensar --Dispensar_IN = '1' and 
		          display <= segmentos_uni_cambio;
		          --display <= segmentos_uni_dinero;
		        else
		          display <= segmentos_uni_dinero;
		        end if;
            when "1101" => -- DISPLAY 2 Activo
		        if ( Confirmar_compra_IN = '1')then --Dispensar_IN = '1' and
		          display <= segmentos_dec_cambio;
		          --display <= segmentos_dec_dinero;
		        else
		          display <= segmentos_dec_dinero;
		        end if;
            when "1011" => -- DISPLAY 3 Activo
		        if (Confirmar_compra_IN = '0')then --and Dispensar_IN = '0' --producto_selected = '1' and 
		          display <= segmentos_uni_precio;
		        elsif ( Confirmar_compra_IN = '1')then -- and Confirmar_compra_IN = '1' --Dispensar_IN = '1' and
		          display <= segmentos_letra;
		        else
		          display <= "1111111"; -- 0
		        end if;
            when "0111" => -- DISPLAY 4 Activo
		        if ( Confirmar_compra_IN = '0')then --producto_selected = '1' and
		          display <= segmentos_dec_precio;
		        elsif ( Confirmar_compra_IN = '1')then --Dispensar_IN = '1'and
		          --display <= segmentos_uni_dinero;
		          display <= "1111111";
		        else
		          display <= "1111111"; -- 0
		        end if; 
            when others =>       
                      --g f e d c b a
                display <= "1111111"; 

	    end case;
	end process;

end Behavioral;
