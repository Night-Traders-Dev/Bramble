# Adding Bramble Logo

To add the official Bramble RP2040 Emulator logo to the repository:

## Option 1: Via GitHub Web Interface (Easiest)

1. Go to: https://github.com/Night-Traders-Dev/Bramble
2. Click "Add file" → "Upload files"
3. Create new folder: `assets`
4. Upload the `bramble-logo.jpg` file to the `assets/` folder
5. Commit with message: "Add Bramble RP2040 Emulator logo - pixel art design"

## Option 2: Via Command Line

```bash
# Clone repository
git clone https://github.com/Night-Traders-Dev/Bramble.git
cd Bramble

# Create assets directory if needed
mkdir -p assets

# Copy logo file
cp /path/to/generated-image.jpg assets/bramble-logo.jpg

# Add and commit
git add assets/bramble-logo.jpg
git commit -m "Add Bramble RP2040 Emulator logo - pixel art design"

# Push to repository
git push origin main
```

## File Details

- **Filename**: `bramble-logo.jpg`
- **Location**: `assets/bramble-logo.jpg`
- **Size**: ~270 KB
- **Format**: JPEG
- **Resolution**: Pixel art design
- **Usage**: Referenced in README.md as project logo

## Verification

After uploading, verify:

1. File is accessible at: `https://raw.githubusercontent.com/Night-Traders-Dev/Bramble/main/assets/bramble-logo.jpg`
2. README.md displays the logo at the top
3. Logo shows as:
   ```markdown
   ![Bramble RP2040 Emulator](assets/bramble-logo.jpg)
   ```

## Logo Features

The official Bramble logo features:
- 🎨 Pixel art style design
- 🟣 Purple/pink color scheme (matching Bramble berry)
- 💻 Computer/emulation theme
- 📱 Monitoring/debugging elements
- ✨ Professional and approachable

Featured badges/icons:
- Open Source (raspberry)
- Lightweight (leaf)
- Accurate (speedometer)
- Cross-Platform (gamepad)

---

**Next Step**: Once logo is uploaded, the README.md will automatically display it at the top of the project page.
