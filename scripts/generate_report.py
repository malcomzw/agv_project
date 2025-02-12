import os
import xml.etree.ElementTree as ET
from fpdf import FPDF
import markdown2
import glob

def markdown_to_pdf(markdown_file, pdf_file):
    # Read markdown content
    with open(markdown_file, 'r') as f:
        markdown_content = f.read()
    
    # Convert markdown to HTML
    html_content = markdown2.markdown(markdown_content)
    
    # Create PDF
    pdf = FPDF()
    pdf.add_page()
    pdf.set_font('Arial', '', 12)
    
    # Write HTML content to PDF (simple conversion)
    pdf.write_html(html_content)
    
    pdf.output(pdf_file)

def create_report():
    # Ensure documentation directory exists
    os.makedirs('documentation', exist_ok=True)
    
    # Convert all markdown files in docs to PDF
    markdown_files = glob.glob('../docs/*.md')
    for md_file in markdown_files:
        filename = os.path.basename(md_file)
        pdf_filename = os.path.join('documentation', filename.replace('.md', '.pdf'))
        markdown_to_pdf(md_file, pdf_filename)
    
    # Optional: Add test results PDF generation
    try:
        pdf = FPDF()
        pdf.add_page()
        pdf.set_font('Arial', 'B', 16)
        
        # Add test results if available
        if os.path.exists('test_results.xml'):
            tree = ET.parse('test_results.xml')
            pdf.cell(200, 10, txt='Test Results', ln=1)
        
        # Add Gazebo performance data if available
        if os.path.exists('simulation.log'):
            with open('simulation.log', 'r') as f:
                gazebo_data = f.read()
            pdf.multi_cell(0, 10, f'Gazebo Model States:\n{gazebo_data}')
        
        pdf.output('documentation/pipeline_report.pdf')
    except Exception as e:
        print(f"Could not generate additional report: {e}")

if __name__ == '__main__':
    create_report()
