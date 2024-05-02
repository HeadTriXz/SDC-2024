<div align="center">

[![Banner SDC][banner]][link-repo]

</div>

<div align="center">
    <h1>Self Driving Challenge - Edition 2024</h1>
    <p>This repository contains the code used by the team of the Hanze during the Self Driving Challenge 2024</p>
</div>

## Table of Contents
- [About](#about)
- [Installation & Usage](#installation--usage)
- [License](#license)

## About
The Self Driving Challenge is an annual competition organized by the RDW, where teams from different universities and colleges in the Netherlands compete against each other in a series of challenges. The goal of the competition is to develop a self-driving car that can navigate through a series of challenges, such as following a line, avoiding obstacles, and recognizing traffic signs.

## Installation & Usage
**Prerequisites**

This project is built using Python. Ensure you have Python 3.12 and its package manager pip installed on your system. Verify their versions in your terminal with these commands:
```bash
python --version
pip --version
```

**Step 1: Clone the Repository**
```bash
git clone https://github.com/HeadTriXz/SDC-2024
```

**Step 2: Create a Virtual Environment**
```bash
python -m venv venv
```

**Step 3: Activate the Virtual Environment**
```bash
# On Windows
venv\Scripts\activate

# On macOS and Linux
source venv/bin/activate
```

**Step 4: Install the Required Packages**
```bash
pip install -r requirements.txt
```

**Step 5: Run the Application**
```bash
python -m src.main
```

### Execute a Script
To execute a script, run the following command in your terminal:
```bash
python -m scripts.python.<script_name>
```

## License
This project is licensed under the **MIT License**.

See the [LICENSE](LICENSE) file for more information.

<!-- Links -->
[link-repo]:https://github.com/HeadTriXz/SDC-2024
[banner]:https://github.com/HeadTriXz/SDC-2024/assets/32986761/e1194707-aa35-4649-b31d-97624179e18f
