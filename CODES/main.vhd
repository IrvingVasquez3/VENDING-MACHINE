-- MAIN 
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity main is
    Port ( 
            clk100Mhz : in std_logic;
            
            UP_IN : in std_logic;
            DOP_IN : in std_logic;
            CP_IN : in std_logic;
            DP_IN : in std_logic;
            RST_IN : in std_logic;
            CONF_COMPRA_IN : in std_logic;
           
            PRODUCT_IN : in std_logic_vector (4 downto 0);
           
            seg_display : out std_logic_vector(6 downto 0);
            select_display : out std_logic_vector(3 downto 0);
           
            --Dispensar_out: out std_logic;
            --Dar_cambio_out: out std_logic;      
            LEDS : out STD_LOGIC_VECTOR (15 downto 0)               
    );
end main;
-------------------------------------------------------ARQUITCTURA-------------------------------------------------------------------------
architecture arq_main of main is
-- signals 
signal sign_clk1hz : std_logic; --para contador de segundo en segundo 
signal sign_clk4hz : std_logic;
signal sign_clk10hz: std_logic;
signal sign_clk50hz : std_logic;
signal sign_clk500hz: std_logic;
signal sign_clk1000hz: std_logic;
signal sign_clkDeboun : std_logic;

signal sign_segmentos_uni_dinero : std_logic_vector (6 downto 0);
signal sign_segmentos_dec_dinero : std_logic_vector (6 downto 0);
signal sign_segmentos_uni_cambio : std_logic_vector (6 downto 0);
signal sign_segmentos_dec_cambio : std_logic_vector (6 downto 0);
signal sign_segmentos_uni_precio : std_logic_vector (6 downto 0);
signal sign_segmentos_dec_precio : std_logic_vector (6 downto 0);
signal sign_segmentos_letra : std_logic_vector (6 downto 0);

signal sign_RecibeDinero : std_logic;
signal sign_DineroRecibido : integer range 0 to 99; 
signal sign_CambioDado : integer range 0 to 99; 
--signal sign_DineroReci_Uni : integer range 0 to 9; --:= 0;
--signal sign_DineroReci_Dec : integer range 0 to 9; --:= 0;

signal sign_product_selected : std_logic;
signal sign_product_precio : integer;

signal UP_IN_SIN_REB : std_logic;
signal DOP_IN_SIN_REB : std_logic;
signal CP_IN_SIN_REB : std_logic;
signal DP_IN_SIN_REB : std_logic;
signal RST_IN_SIN_REB : std_logic;
signal CONF_COMPRA_IN_SIN_REB : std_logic;
signal PRODUCT_IN_SIN_REB :  std_logic_vector (4 downto 0);

signal signal_Dispensar_out : std_logic;
signal signal_Dar_cambio_out : std_logic;

-------------------------------------------------components---------------------------------
component div_freq is
    generic( max_conteo : integer);
    -- 100 MHz (clk de fpga) / 10 Hz (freq requerida) = (conteo Maximo)
   
    -- 1 ciclo por segundo 1 Hz --1 hz
    -- 100 MHz / 1 Hz = 100 000 000
    --conteo_max = 100000000;
    
    -- 500 ciclos por seg o 1 ciclo cada 2 ms -- 500 hz
    -- 100 MHz / 500 Hz = 200 000 
    -- conteo max de 200000
  
    -- 4 ciclos por seg o 1 ciclo cada 0.25 s -- 4 hz
    -- 100 MHz / 4 Hz = 25 000 000
    -- conteo max de 25000000
    port (
        clkOriginal : in std_logic;
        reset : in std_logic;
        clkDividido : out std_logic
    );
end component;

--component contadorNbits is
    --generic( N: integer); 
    --port(
        --clk_100MHz_contador : in std_logic; --cuenta los pulsos de reloj
        --reset: in std_logic;    
        --count_enable: in std_logic;    --count enable
        --terminal_count: out std_logic;    --teminal count
      --  counter_out: out std_logic_vector (N - 1 downto 0);
    --    counter_out_int: out integer  
  --  );
--end component;

component debouncer is
    port ( 
        clk_100MHz : in std_logic; 
        btn_in : in std_logic;
        btn_out : out std_logic;
        btn_pulsed : out std_logic
    );
end component;

component suma_dinero is
    port( 
        clk_100mhz : in std_logic;
           
        UP_IN : in std_logic;
        DOP_IN : in std_logic;
        CP_IN : in std_logic;
        DP_IN : in std_logic;
        RST_IN : in std_logic;
            
        DineroReci : out integer range 0 to 99;
        RecibeDinero : out std_logic;
        Entregar_dinero: in std_logic;
        precio : in integer;
        dinero : in integer;
        estado_dispensar : in std_logic;
        Dar_cambio : in std_logic
    );
end component;

component divNum_Uni_Dec is
    port ( 
        Number : in integer range 0 to 99;
        segmentos_uni_out : out std_logic_vector (6 downto 0);
        segmentos_dec_out : out std_logic_vector (6 downto 0)
    );
end component;



component demux_display is
    port (
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
end component;

component seleccionar_producto is
    port ( 
        clk : in std_logic;
        rst : in std_logic;        
        PRODUCT_1 : in std_logic;
        PRODUCT_2 : in std_logic;
        PRODUCT_3 : in std_logic;
        PRODUCT_4 : in std_logic;
        PRODUCT_5 : in std_logic;
        CONFIRMAR_COMPRA : in std_logic;
        producto_selected : out std_logic;
        precio_producto : out integer
    );
end component;

component estados_dispensador is
    port ( 
        clk: in std_logic;
        rst: in std_logic;
        productoSelected : in std_logic;
        confirmarProductoSelected : in std_logic;
        precioProducto: in integer;
        dineroRecibido: in integer;
        Dispensar: out std_logic;
        Dar_cambio: out std_logic
    );
end component;

component mostrar_dispensar_leds is
Port ( 
    clk : in STD_LOGIC;
    reset : in STD_LOGIC;
    Confirmar_compra_IN : std_logic; 
    empezarCuenta : in STD_LOGIC;
    LEDS : out STD_LOGIC_VECTOR (15 downto 0)
);
end component;

component obtener_cambio is
Port ( 
    clk_100mhz : in std_logic;
    Dispensar_out : in std_logic;
    Confirmar : in std_logic;
    dineroAcumulado : in integer;
    precioProducto : in integer;
    valor_del_cambio : out integer
);
end component;

-----------------------------------------------------BEGIN-------------------------------------------
begin
    clk_1Hz:    div_freq 
                generic map( max_conteo => 100000000)
                port map(
                    clkOriginal => clk100Mhz,
                    reset => RST_IN,
                    clkDividido  => sign_clk1hz
    );

    clk_4Hz:  div_freq 
    -- contador 4 Hz. conteoMax= 25000000. T=0.25s . 
                generic map( max_conteo => 25000000)
                port map(
                    clkOriginal => clk100Mhz,
                    reset => RST_IN,
                    clkDividido  => sign_clk4hz
    ); 
    
    clk_500Hz:  div_freq 
                generic map( max_conteo => 200000)
                port map(
                    clkOriginal => clk100Mhz,
                    reset => RST_IN,
                    clkDividido  => sign_clk500hz
    ); 
    
    clk_10Hz:  div_freq 
    -- contador 1000 Hz. conteoMax= 100000. T=0.001=1ms . Cada 2 ms  cambia 
    -- contador 10 Hz. conteoMax= 10000000 
                generic map( max_conteo => 10000000)
                port map(
                    clkOriginal => clk100Mhz,
                    reset => RST_IN,
                    clkDividido  => sign_clk10hz
    ); 
    
    clk_1000Hz:  div_freq 
    -- contador 1000 Hz. conteoMax= 100000. T=0.001=1ms . Cada 2 ms  cambia 
    -- contador 10 Hz. conteoMax= 100000. T=0.001=1ms . Cada 2 ms  cambia 
                generic map( max_conteo => 100000)
                port map(
                    clkOriginal => clk100Mhz,
                    reset => RST_IN,
                    clkDividido  => sign_clk1000hz
    ); 
    
    UP_btn: debouncer    port map (
            clk_100MHz => clk100Mhz,
            btn_in => UP_IN,
            btn_pulsed => UP_IN_SIN_REB --out
    );
     
    DOP_btn: debouncer    port map (
            clk_100MHz => clk100Mhz,
            btn_in => DOP_IN,
            btn_pulsed => DOP_IN_SIN_REB --out
    );
    CP_btn: debouncer    port map (
            clk_100MHz => clk100Mhz,
            btn_in => CP_IN,
            btn_pulsed => CP_IN_SIN_REB --out
    );
    DP_btn: debouncer    port map (
            clk_100MHz => clk100Mhz,
            btn_in => DP_IN,
            btn_pulsed => DP_IN_SIN_REB --out
    );
    RST_btn: debouncer    port map (
            clk_100MHz => clk100Mhz,
            btn_in => RST_IN,
            btn_pulsed => RST_IN_SIN_REB --out
    );    
    Product_1_btn: debouncer port map (
            clk_100MHz => clk100Mhz,
            btn_in => PRODUCT_IN(0),
            btn_pulsed => PRODUCT_IN_SIN_REB(0)
    );
    Product_2_btn: debouncer port map (
            clk_100MHz => clk100Mhz,
            btn_in => PRODUCT_IN(1),
            btn_pulsed => PRODUCT_IN_SIN_REB(1)
    );
    Product_3_btn: debouncer port map (
            clk_100MHz => clk100Mhz,
            btn_in => PRODUCT_IN(2),
            btn_pulsed => PRODUCT_IN_SIN_REB(2)
    );
    Product_4_btn: debouncer port map (
            clk_100MHz => clk100Mhz,
            btn_in => PRODUCT_IN(3),
            btn_pulsed => PRODUCT_IN_SIN_REB(3)
    );
    Product_5_btn: debouncer port map (
            clk_100MHz => clk100Mhz,
            btn_in => PRODUCT_IN(4),
            btn_pulsed => PRODUCT_IN_SIN_REB(4)
    );
    
    confirmarCompra_btn: debouncer port map (
            clk_100MHz => clk100Mhz,
            btn_in => CONF_COMPRA_IN,
            btn_pulsed => CONF_COMPRA_IN_SIN_REB
    );
    
    sumar_monedas: suma_dinero port map(
            clk_100mhz  => clk100Mhz, -- clk
        
            UP_IN => UP_IN_SIN_REB,
    	    DOP_IN => DOP_IN_SIN_REB,
            CP_IN => CP_IN_SIN_REB,
            DP_IN => DP_IN_SIN_REB,
            RST_IN => RST_IN_SIN_REB,
            
            DineroReci => sign_DineroRecibido,
            RecibeDinero => sign_RecibeDinero,
            Entregar_dinero => CONF_COMPRA_IN, --CONF_COMPRA_IN_SIN_REB
            precio => sign_product_precio,
            dinero => sign_product_precio,
            estado_dispensar => signal_Dispensar_out,
            Dar_cambio => signal_Dar_cambio_out            
    );
    
    DineroIngresado: divNum_Uni_Dec port map(
                    Number => sign_DineroRecibido,
                    segmentos_uni_out => sign_segmentos_uni_dinero,
                    segmentos_dec_out => sign_segmentos_dec_dinero                    
    );
    
    select_prod: seleccionar_producto port map ( 
        clk => clk100Mhz,
        rst => RST_IN_SIN_REB,
        PRODUCT_1 => PRODUCT_IN(0),
        PRODUCT_2 => PRODUCT_IN(1),
        PRODUCT_3 => PRODUCT_IN(2),
        PRODUCT_4 => PRODUCT_IN(3),
        PRODUCT_5 => PRODUCT_IN(4),
        CONFIRMAR_COMPRA => CONF_COMPRA_IN,--CONF_COMPRA_IN_SIN_REB
        producto_selected => sign_product_selected,
        precio_producto => sign_product_precio
    );
    precio_productoSel: divNum_Uni_Dec port map(
                    Number => sign_product_precio,
                    segmentos_uni_out => sign_segmentos_uni_precio,
                    segmentos_dec_out => sign_segmentos_dec_precio                    
    );
    
    disp_producto1: estados_dispensador port map(
       clk => clk100Mhz,
       rst => RST_IN_SIN_REB,
       productoSelected => sign_product_selected,
       confirmarProductoSelected => CONF_COMPRA_IN_SIN_REB,
       precioProducto => sign_product_precio,
       dineroRecibido => sign_DineroRecibido,
       Dispensar => signal_Dispensar_out,
       Dar_cambio => signal_Dar_cambio_out
    );
    dar_cambio : obtener_cambio port map( 
        clk_100mhz => clk100Mhz,
        Dispensar_out => signal_Dispensar_out,
        Confirmar => CONF_COMPRA_IN_SIN_REB,
        dineroAcumulado => sign_DineroRecibido,
        precioProducto => sign_product_precio,
        valor_del_cambio => sign_CambioDado
    );
    
    Cambio_dado: divNum_Uni_Dec port map(
                    Number => sign_CambioDado,
                    segmentos_uni_out => sign_segmentos_uni_cambio,
                    segmentos_dec_out => sign_segmentos_dec_cambio                    
    );
    
    encender_Lds : mostrar_dispensar_leds
            port map (
            --necesita 10hz
                clk => sign_clk10hz, --sign_clk4hz,
                reset => RST_IN_SIN_REB,
                empezarCuenta => signal_Dispensar_out,
                Confirmar_compra_IN => CONF_COMPRA_IN,
                LEDS => LEDS 
            );    

sign_segmentos_letra <= "1000110";
salida_display: demux_display port map(
                    clk1000hz => sign_clk1000hz,
                    Dispensar_IN => signal_Dispensar_out,
                    Dar_cambio_IN => signal_Dar_cambio_out,
                    Confirmar_compra_IN => CONF_COMPRA_IN,
                    producto_selected => sign_product_selected,
                    segmentos_uni_dinero => sign_segmentos_uni_dinero,
                    segmentos_dec_dinero => sign_segmentos_dec_dinero,
                    segmentos_uni_cambio => sign_segmentos_uni_cambio,
                    segmentos_dec_cambio => sign_segmentos_dec_cambio,
                    segmentos_uni_precio => sign_segmentos_uni_precio,
                    segmentos_dec_precio => sign_segmentos_dec_precio,
                    segmentos_letra => sign_segmentos_letra,
                    display => seg_display,
                    curr_display => select_display
    );

end arq_main;