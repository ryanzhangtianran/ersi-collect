"""Generate a collection bundle; never start the simulator or collect data."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.eval_recollection import convert_failure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", required=True, help="Axis results JSON or snapshot outcome.json")
    parser.add_argument("--episode-id", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--assets-root", required=True, help="Local geniesim_assets directory")
    parser.add_argument("--runtime-assets-root", default="/geniesim_assets")
    parser.add_argument("--robot-cfg", default="G2_omnipicker_fixed_dual.json")
    args = parser.parse_args()
    path = convert_failure(args.result, args.episode_id, args.output_dir, args.assets_root,
                           args.robot_cfg, args.runtime_assets_root)
    print(f"Prepared collection task: {path}")
    print("No collection or training was started. Review rsi_recollection.differences before running.")


if __name__ == "__main__":
    main()
