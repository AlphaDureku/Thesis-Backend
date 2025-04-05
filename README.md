# Route Calculation API

This API calculates routes using A* search with k-step lookahead and traffic data from Google Distance Matrix API. It uses OpenStreetMap for road network data.

## Setup

1. **Download Road Network Graph:**
   - Download `philippines.graphml` from [Google Drive](https://drive.google.com/file/d/1LOGYSwOpKLSOcvMo5xr6bCivZU8S8a8i/view?usp=sharing) and place it in the project directory.

2. **Install Dependencies:**
   - Create a virtual environment (optional):
     ```bash
     python -m venv venv
     ```
   - Activate the virtual environment:
     - Windows:
       ```bash
       venv\Scripts\activate
       ```
     - macOS/Linux:
       ```bash
       source venv/bin/activate
       ```
   - Install required packages:
     ```bash
     pip install -r requirements.txt
     ```

3. **Set Google API Key:**
   - Create a `.env` file and add your Google API key:
     ```
     GOOGLE_API_KEY=your-api-key
     ```

## Run the Application

Start the server:
```bash
python backend.py
