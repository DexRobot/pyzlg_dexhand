import argparse
from typing import List
from IPython import embed
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from dexhand_interface import LeftDexHand, RightDexHand, ControlMode, ZCANWrapper

def initialize_hands(hand_names: List[str]) -> dict:
    """Initialize specified hands and return a dictionary of instances"""
    hands_dict = {}
    zcan = None

    if len(hand_names) > 1:
        # Create shared ZCAN instance if using both hands
        zcan = ZCANWrapper()
        if not zcan.open():
            print("Failed to open shared ZCAN instance")
            return {}

    for name in hand_names:
        if name == 'left':
            hand = LeftDexHand(zcan)
        elif name == 'right':
            hand = RightDexHand(zcan)
        else:
            raise ValueError(f"Unknown hand name: {name}")

        if not hand.init():
            print(f"Failed to initialize {name} hand")
            continue

        print(f"Initialized {name} hand")
        hands_dict[name] = hand

    return hands_dict

def main():
    parser = argparse.ArgumentParser(description="Interactive dexterous hand control")
    parser.add_argument('--hands', nargs='+', choices=['left', 'right'], default=['left'],
                        help='Which hands to initialize (default: left)')
    args = parser.parse_args()

    # Initialize hands
    hands = initialize_hands(args.hands)

    if not hands:
        print("No hands initialized successfully. Exiting.")
        return

    # Create globals dict for IPython
    globals_dict = {
        'ControlMode': ControlMode,
        'hands': hands,
    }
    for i, hand in enumerate(hands):
        globals_dict[f'{args.hands[i]}_hand'] = hands[hand]

    print("Hands initialized. Entering IPython shell...")
    print("Available globals:")
    print(f"  hands: List of initialized hand instances")
    for name in args.hands:
        print(f"  {name}_hand: {name.title()} hand instance")
    print(f"  ControlMode: Enum of possible control modes")
    print("Example commands:")
    print("  left_hand.move_joints(finger_spread=30)")
    print("  right_hand.move_joints(thumb_rot=100, control_mode=ControlMode.PROTECT_HALL_POSITION)")
    print("  left_hand.reset_joints()")
    print("  right_hand.clear_errors()")

    embed(user_ns=globals_dict)

    # Cleanup
    print("Closing hands...")
    for hand in hands:
        hand.close()
    print("Exiting")

if __name__ == '__main__':
    main()
