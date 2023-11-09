# ** worm_multiplayer_messages **

## Messages:
1. PlayerInput:
	- Aufbau:
	  ('x', 'y', 'WormID')
	  Bsp.: ('-2', '0', '69')
	- Kooridinaten (x und y Achse mit jeweils +/-) --> int8 [Werte: -128 bis 127]
	- WormID (postive Werte: pot. Spieler) --> int8

2. Board:
	- Aufbau:
	  ( 'Was es anzeigt', 'ID um zu sagen was es ist')  
	  Bsp.: ('#', '-1')
	- Mögliche Elemete:

	  | Zeichen | Was ist es | Element_ID: |
	  |---------|------------|-------------|
	  | 0 | Wurmkopf | 1 |
	  | o | Wurmkörper | 2 |
	  | # | Barriere | 3 |
	  | weiter Ideen gerne gesehen | | |
	- Zeichen: string (==! Achtung !== Bei speziellen Zeichen aufpassen, da lieber Unicode)
	- Element_ID: positive Zahlen --> int8
