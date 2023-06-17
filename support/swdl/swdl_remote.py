import os
import sys

# main function
def main():
    binary = sys.argv[1]
    print("Binary: " + binary)

    # Scp the binary to the remote machine
    print("Copying binary to remote machine...")
    os.system(f"scp {binary} ripxorip@olirover.local:~/dev/OliRover/support/swdl/_input/input.elf")
    # Run the remote script
    print("Performing the download remotely...")
    os.system(f"ssh ripxorip@olirover.local 'python ~/dev/OliRover/support/swdl/swdl.py ~/dev/OliRover/support/swdl/_input/input.elf'")

if __name__ == "__main__":
    main()