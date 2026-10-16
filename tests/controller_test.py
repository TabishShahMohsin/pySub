import os

import pygame


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("❌ No controller found.")
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()

    print(f"📡 Debugging: {joy.get_name()}")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            pygame.event.pump()

            # Clear terminal screen for a clean live view
            os.system("cls" if os.name == "nt" else "clear")

            print(f"--- Controller: {joy.get_name()} ---")

            # 1. Check ALL Axes (Sticks and Triggers)
            print("\nAXES (Sticks/Triggers):")
            for i in range(joy.get_numaxes()):
                val = joy.get_axis(i)
                # Draw a simple progress bar for visualization
                bar = "#" * int((val + 1) * 10)
                print(f"Axis {i}: [{bar:<20}] {val:>6.3f}")

            # 2. Check ALL Buttons
            print("\nBUTTONS:")
            btn_states = []
            for i in range(joy.get_numbuttons()):
                state = joy.get_button(i)
                btn_states.append(f"B{i}: {'ON ' if state else 'OFF'}")

            # Print buttons in rows of 4 for readability
            for i in range(0, len(btn_states), 4):
                print("  ".join(btn_states[i : i + 4]))

            pygame.time.wait(100)  # 10Hz update rate

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
