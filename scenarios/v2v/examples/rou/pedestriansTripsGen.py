import xml.etree.ElementTree as ET
import random
import sys
from xml.dom import minidom

# List of pedestrian types
pedestrian_types = [f"walker.pedestrian.{i:04d}" for i in range(1, 33)]

# List of all edges
all_edges = [
    "E26.14", "E27", "E28.12", "E3", "E28.16", "E24", "E27.16", "-E3", "-E2"
]

# Edges that cannot be destination
invalid_end_edges = {"E0", "E1"}

def generate_pedestrians(num_pedestrians, start_depart_time, depart_increment):
    if num_pedestrians < 1:
        print("Error: Number of pedestrians must be at least 1.")
        sys.exit(1)
    
    # Create the root element
    root = ET.Element("routes", xmlns_xsi="http://www.w3.org/2001/XMLSchema-instance", xsi_noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd")
    
    # List of valid edges for destination
    valid_end_edges = [edge for edge in all_edges if edge not in invalid_end_edges]
    
    for i in range(num_pedestrians):
        # Randomly select a pedestrian type
        pedestrian_type = random.choice(pedestrian_types)
        
        # Randomly select a starting edge (excluding E0 and E1)
        start_edge = random.choice(all_edges)
        while start_edge in invalid_end_edges:
            start_edge = random.choice(all_edges)
        
        # Randomly select a destination edge (excluding E0 and E1)
        destination_edge = random.choice(valid_end_edges)
        
        # Calculate the departure time (starting at 'start_depart_time' and incrementing by 'depart_increment')
        depart_time = start_depart_time + i * depart_increment
        
        # Create a person element with the chosen type and departure time
        person = ET.Element("person", id=f"person{i}", type=pedestrian_type, depart=f"{depart_time:.2f}")
        
        # Use a dictionary for attributes to avoid syntax issues
        person_trip_attrs = {
            "from": start_edge,
            "to": destination_edge,
            "walkFactor": "0.75"
        }
        
        person_trip = ET.SubElement(person, "personTrip", **person_trip_attrs)
        root.append(person)
    
    # Write to the output file with spaces between pedestrians
    output_filename = "pedestrians.rou.xml"
    
    # Convert the XML tree to a string with indentation
    xml_str = ET.tostring(root, 'utf-8')
    parsed = minidom.parseString(xml_str)
    pretty_xml_str = parsed.toprettyxml(indent="  ")
    
    # Write the formatted XML to the file
    with open(output_filename, "w") as file:
        file.write(pretty_xml_str)
    
    print(f"XML file '{output_filename}' created with {num_pedestrians} pedestrians.")

if __name__ == "__main__":
    # Check for the correct number of arguments
    if len(sys.argv) != 4:
        print("Usage: python pedestriansTripsGen.py <num_pedestrians> <start_depart_time> <depart_increment>")
        sys.exit(1)
    
    # Parse command-line arguments
    num_pedestrians = int(sys.argv[1])
    start_depart_time = float(sys.argv[2])  # Starting departure time
    depart_increment = float(sys.argv[3])   # Increment between pedestrians' departure times
    
    generate_pedestrians(num_pedestrians, start_depart_time, depart_increment)
