import xml.etree.ElementTree as ET
import random
import sys
from xml.dom import minidom

# List of vehicle type IDs
vehicle_types = [
    "vehicle.audi.a2",
    "vehicle.audi.etron",
#    "vehicle.audi.tt",
#    "vehicle.bh.crossbike",
    "vehicle.bmw.grandtourer",
#    "vehicle.carlamotors.carlacola",
    "vehicle.chevrolet.impala",
    "vehicle.citroen.c3",
#    "vehicle.diamondback.century",
    "vehicle.dodge.charger_police",
    "vehicle.ford.mustang",
#    "vehicle.gazelle.omafiets",
#    "vehicle.harley-davidson.low_rider",
    "vehicle.jeep.wrangler_rubicon",
#    "vehicle.kawasaki.ninja",
    "vehicle.lincoln.mkz_2017",
    "vehicle.mercedes.coupe",
    "vehicle.micro.microlino",
    "vehicle.mini.cooper_s",
    "vehicle.nissan.micra",
    "vehicle.nissan.patrol",
    "vehicle.seat.leon",
#    "vehicle.tesla.cybertruck",
    "vehicle.tesla.model3",
    "vehicle.toyota.prius",
    "vehicle.volkswagen.t2"
]

# Parking area associations with edges
edge_parking_areas = {
    "E28.12": [f"pa_{i}" for i in range(5, 10)],
    "E24": [f"pa_{i}" for i in range(0, 5)],
    "E1": [f"pa_{i}" for i in range(10, 30)],
    "E0": [f"pa_{i}" for i in range(30, 50)],
    "E27": [f"pa_{i}" for i in range(50, 55)],
    "E27.16": [f"pa_{i}" for i in range(55, 60)]
}

def create_vehicle_xml(num_vehicles, depart_increment):
    if num_vehicles > 60:
        print("Error: Cannot create more than 60 vehicles.")
        sys.exit(1)
    
    # Create the root element
    root = ET.Element("routes", xmlns_xsi="http://www.w3.org/2001/XMLSchema-instance", xsi_noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd")
    
    # Initialize parking area pool
    available_parking_areas = [pa for parking_areas in edge_parking_areas.values() for pa in parking_areas]
    
    if num_vehicles > len(available_parking_areas):
        print("Error: Number of vehicles exceeds available parking areas.")
        sys.exit(1)

    # Shuffle parking areas and select num_vehicles number of them
    random_parking_areas = random.sample(available_parking_areas, num_vehicles)
    
    for i in range(num_vehicles):
        # Randomly select a vehicle type
        vehicle_type = random.choice(vehicle_types)
        
        # Assign a random parking area
        parking_area = random_parking_areas[i]
        
        # Find the edge corresponding to the parking area
        edge = None
        for e, areas in edge_parking_areas.items():
            if parking_area in areas:
                edge = e
                break
        
        if edge is None:
            print(f"Error: No edge found for parking area {parking_area}.")
            sys.exit(1)
        
        # Set departure time based on the increment argument
        depart_time = i * depart_increment
        
        # Create a vehicle element with the chosen type and parking area
        vehicle = ET.Element("vehicle", id=f"car{i}", depart=f"{depart_time:.2f}", type=vehicle_type, departPos="stop")
        route = ET.SubElement(vehicle, "route", edges=edge)  # Corrected edges to the associated edge
        stop = ET.SubElement(vehicle, "stop", parkingArea=parking_area, duration="150")
        root.append(vehicle)
    
    # Write to the output file with spaces between vehicles
    output_filename = "vehicles.rou.xml"
    
    # Convert the XML tree to a string with indentation
    xml_str = ET.tostring(root, 'utf-8')
    parsed = minidom.parseString(xml_str)
    pretty_xml_str = parsed.toprettyxml(indent="  ")
    
    # Write the formatted XML to the file
    with open(output_filename, "w") as file:
        file.write(pretty_xml_str)
    
    print(f"XML file '{output_filename}' created with {num_vehicles} vehicles.")

if __name__ == "__main__":
    # Check for the correct number of arguments
    if len(sys.argv) != 3:
        print("Usage: python generate_vehicles.py <num_vehicles> <depart_increment>")
        sys.exit(1)
    
    # Parse command-line arguments
    num_vehicles = int(sys.argv[1])
    depart_increment = float(sys.argv[2])  # Time increment between vehicle departures
    
    create_vehicle_xml(num_vehicles, depart_increment)
