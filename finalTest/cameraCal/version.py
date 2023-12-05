import cv2

# Print the version of OpenCV
print("OpenCV version:", cv2.__version__)

# Check for contrib modules
contrib_modules = [name for name in dir(cv2) if name.startswith('cv2.cv2')]
for module in contrib_modules:
    version = getattr(cv2, module).__version__ if hasattr(getattr(cv2, module), '__version__') else "No version information"
    print(f"{module}: {version}")
