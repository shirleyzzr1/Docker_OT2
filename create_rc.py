import yaml
from argparse import ArgumentParser


file_path = "~/config/temp/rc_document.yaml"

def parse_arguments():
    parser = ArgumentParser()
    
    parser.add_argument(
        "-ip", 
        "--robot_ip",
        help = "[REQUIRED] Wired IP address of the OT2 Robot",
        # default= "255.255.255.255",
        required=True
    )

    parser.add_argument(
        "-rc",
        "--robot-config-path",
        default="/root/config/temp/rc_document.yaml",
        help="The absolute path destination of the robot config file"
    )

    # parser.p
    return parser.parse_args()


def generate_yaml(ip_address, file_destination):
    
    with open(file_destination, "w") as file:
        yaml.dump([{"ip": ip_address }], file)

if __name__ == "__main__":
    args = parse_arguments()
    print(args)
    generate_yaml(args.robot_ip, args.robot_config_path)
    