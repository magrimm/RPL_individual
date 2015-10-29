note
	description: "Controller for Features like Colour and Sound."
	author: "Maruis Grimm"
	date: "05.10.15"

class
	FEATURE_CONTROLLER

create
	make

feature {NONE} -- Initialization

	make
		do
			create sound_player.make_with_topic ({THYMIO_TOPICS}.sound)
			create top_leds.make_with_topic ({THYMIO_TOPICS}.top_leds)
			create buttons_leds.make_with_topic ({THYMIO_TOPICS}.buttons_leds)
			create circle_leds.make_with_topic ({THYMIO_TOPICS}.circle_leds)
		end

feature -- Access

	features_go_to_goal
		-- Run features for "go_to_goal".
		do
			light_up_leds (top_leds, buttons_leds, circle_leds)
			sound (3, sound_player)
		end

	features_goal_reached
		-- Run different features when goal reached.
		do
			light_down_leds (top_leds, buttons_leds, circle_leds)
			sound (5, sound_player)
		end

feature {NONE} -- Robot parts

	sound_player: separate THYMIO_SOUND_PLAYER
			-- Built-in sound player.

	top_leds: separate THYMIO_TOP_LEDS
			-- RGB LEDs on the top.

	buttons_leds: separate THYMIO_BUTTONS_LEDS
			-- 4 Red LEDs on the buttons.

	circle_leds: separate THYMIO_CIRCLE_LEDS
			-- 8 Orange LEDS around the buttons.

feature {NONE} -- Behaviours

	light_up_leds (top: separate THYMIO_TOP_LEDS;
					buttons: separate THYMIO_BUTTONS_LEDS;
					circle: separate THYMIO_CIRCLE_LEDS)
			-- Turn on LED lights.
		do
			-- Set Top LEDS to yellow
			top.set_to_yellow
			-- Turn on Buttons and Circle with full brightness.
			buttons.set_leds_brightness (<<32,32,23,32>>)
			circle.set_leds_brightness (<<32,32,32,32,32,32,32,32>>)
		end

	light_down_leds (top: separate THYMIO_TOP_LEDS;
					buttons: separate THYMIO_BUTTONS_LEDS;
					circle: separate THYMIO_CIRCLE_LEDS)
			-- Change LED lights.
		do
			top.set_to_green
			buttons.set_leds_brightness (<<0,0,0,0>>)
			circle.set_leds_brightness (<<0,0,0,0,0,0,0,0>>)
		end

	sound (tone: INTEGER_16; player: separate SOUND_PLAYER)
			-- Play sound
		do
			player.stop_sound
			player.start_system_sound (tone)
		end

end -- class FEATURE_CONTROLLER
