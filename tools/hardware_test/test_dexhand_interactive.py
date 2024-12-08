import argparse
from typing import List
from IPython import embed
import sys
import os
from pyzlg_dexhand.dexhand_interface import LeftDexHand, RightDexHand, ControlMode, ZCANWrapper

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

    print("\nAvailable Joints:")
    print("  Thumb:")
    print("    th_rot  - Thumb rotation (0-150 degrees)")
    print("    th_mcp  - Thumb metacarpophalangeal flexion (0-90 degrees)")
    print("    th_dip  - Thumb distal joints coupled flexion (0-90 degrees)")
    print("  Fingers:")
    print("    ff_spr  - Four-finger spread/abduction (0-30 degrees)")
    print("    ff_mcp  - Index finger metacarpophalangeal flexion (0-90 degrees)")
    print("    ff_dip  - Index finger distal joints flexion (0-90 degrees)")
    print("    mf_mcp  - Middle finger metacarpophalangeal flexion (0-90 degrees)")
    print("    mf_dip  - Middle finger distal joints flexion (0-90 degrees)")
    print("    rf_mcp  - Ring finger metacarpophalangeal flexion (0-90 degrees)")
    print("    rf_dip  - Ring finger distal joints flexion (0-90 degrees)")
    print("    lf_mcp  - Little finger metacarpophalangeal flexion (0-90 degrees)")
    print("    lf_dip  - Little finger distal joints flexion (0-90 degrees)")

    print("\nExample Commands:")
    print("  Move joints:")
    print(f"    {args.hands[i]}_hand.move_joints(th_rot=30, th_mcp=45)        # Move thumb")
    print(f"    {args.hands[i]}_hand.move_joints(ff_spr=20)                   # Spread fingers")
    print(f"    {args.hands[i]}_hand.move_joints(ff_mcp=90, ff_dip=90, control_mode=ControlMode.PROTECT_HALL_POSITION)       # Curl index finger using the protected hall position control mode")

    print("\n  Other commands:")
    print(f"    {args.hands[i]}_hand.reset_joints()           # Move all joints to zero position")
    print(f"    {args.hands[i]}_hand.get_feedback()           # Get current joint and tactile feedback")
    print(f"    {args.hands[i]}_hand.clear_all_errors()           # Clear any error states")

    embed(user_ns=globals_dict)

    # Cleanup
    print("Closing hands...")
    for hand in hands:
        hand.close()
    print("Exiting")

if __name__ == '__main__':
    main()
