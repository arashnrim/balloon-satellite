import io
from pathlib import Path

while True:
    input_file = input("Enter the full path of the recording file: ")
    output_directory = input(
        "Enter the (existing) directory to save the outputs: ")

    with io.open(input_file, encoding="utf-8") as file:
        lines = file.readlines()

        # Creates a new directory to store the files
        try:
            Path("{}/recording_output".format(output_directory)).mkdir()
        except FileNotFoundError:
            print("Check your output directory again; it should be existing.\n")
            continue
        except FileExistsError:
            print("Files found, proceed?", end=" ")
            proceed = bool(
                input("Leave blank to end program, type anything to continue: "))
            if not proceed:
                break

        # Saves humidity values into a separate file
        with io.open("{}/recording_output/humidity.txt".format(output_directory), "w",
                     encoding="utf-8") as humidity_file:
            for line in lines:
                try:
                    if line.split()[0] == "Humidity:":
                        humidity_file.write(
                            "{}\n".format(line.split()[1][:-1]))
                except IndexError:
                    pass
            print("Humidity data extracted.")

        # Saves temperature values into a separate file
        with io.open("{}/recording_output/temperature.txt".format(output_directory), "w",
                     encoding="utf-8") as temp_file:
            for line in lines:
                try:
                    if line.split()[0] == "Temperature:":
                        temp_file.write("{}\n".format(line.split()[1][:-2]))
                except IndexError:
                    pass
            print("Temperature data extracted.")

        # Saves pressure values into a separate file
        with io.open("{}/recording_output/pressure.txt".format(output_directory), "w",
                     encoding="utf-8") as pressure_file:
            for line in lines:
                try:
                    if line.split()[0] == "Pressure:":
                        pressure_file.write(
                            "{}\n".format(line.split()[1][:-3]))
                except IndexError:
                    pass
            print("Pressure data extracted.")

        axes = ["x-axis", "y-axis", "z-axis"]
        # Saves 9DOF values into a separate file
        for axis in axes:
            with io.open("{}/recording_output/{}_accel.txt".format(output_directory, axis), "w",
                         encoding="utf-8") as accel_file:
                for line in lines:
                    try:
                        if line.split()[0] == axis and line.split()[1] == "acceleration:":
                            accel_file.write(
                                "{}\n".format(line.split()[2][:-5]))
                    except IndexError:
                        pass
                print("{} acceleration data extracted.".format(axis.capitalize()))

            with io.open("{}/recording_output/{}_magnet.txt".format(output_directory, axis), "w",
                         encoding="utf-8") as magnet_file:
                for line in lines:
                    try:
                        if line.split()[0] == axis and line.split()[1] == "magnetometer:":
                            magnet_file.write(
                                "{}\n".format(line.split()[2][:-2]))
                    except IndexError:
                        pass
                print("{} magnetometer data extracted.".format(axis.capitalize()))

            with io.open("{}/recording_output/{}_gyro.txt".format(output_directory, axis), "w",
                         encoding="utf-8") as gyro_file:
                for line in lines:
                    try:
                        if line.split()[0] == axis and line.split()[1] == "gyroscope:":
                            gyro_file.write(
                                "{}\n".format(line.split()[2][:-5]))
                        elif len(line.split()) == 1:
                            gyro_file.write(
                                "{}\n".format(line.split()[0][:-5]))
                    except IndexError:
                        pass
                print("{} gyroscope data extracted.".format(axis.capitalize()))

        break

print("Operation concluded successfully.")
