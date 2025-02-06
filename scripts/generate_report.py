import os
import xml.etree.ElementTree as ET
from fpdf import FPDF

def create_report():
    pdf = FPDF()
    pdf.add_page()
    pdf.set_font('Arial', 'B', 16)
    
    # Add test results
    tree = ET.parse('test_results.xml')
    pdf.cell(200, 10, txt='Test Results', ln=1)
    
    # Add pipeline metrics
    # Add Gazebo performance data
    with open('simulation.log', 'r') as f:
        gazebo_data = f.read()
    pdf.multi_cell(0, 10, f'Gazebo Model States:\n{gazebo_data}')
    
    pdf.output('pipeline_report.pdf')

if __name__ == '__main__':
    create_report()
