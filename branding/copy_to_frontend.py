"""
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
03.08.26, 12:57

Copies the required image variants from the
generated folder to the frontend folders in the application.
"""

from pathlib import Path


if __name__ == "__main__":
    generated = Path(__file__).parent.resolve() / "generated"
    ui = generated.parent.parent / "packages" / "frontend" / "ui"

    # assets
    (generated / "color" / "apple-touch-icon-dark.png").copy_into(ui / "assets" / "logo" / "icon")
    (generated / "color" / "apple-touch-icon-light.png").copy_into(ui / "assets" / "logo" / "icon")
    (generated / "color" / "favicon-32.png").copy_into(ui / "assets" / "logo" / "icon")
    (generated / "color" / "favicon-192.png").copy_into(ui / "assets" / "logo" / "icon")
    (generated / "color" / "favicon-512.png").copy_into(ui / "assets" / "logo" / "icon")
    (generated / "color" / "favicon.ico").copy_into(ui / "assets" / "logo" / "icon")
    (generated / "color" / "favicon.svg").copy_into(ui / "assets" / "logo" / "icon")
    (generated / "colorless" / "storage_logo_colorless.svg").copy_into(ui / "assets" / "logo" / "icon")

    # public
    (generated / "color" / "apple-touch-icon.png").copy_into( ui / "public")
    (generated / "color" / "favicon.ico").copy_into( ui / "public")
    # these are for webmanifest, which we can't easily generate so we can't use assets there
    (generated / "color" / "storage_logo_color_0128.png").copy(ui / "public" / "icon-128.svg")
    (generated / "color" / "storage_logo_color_0192.png").copy(ui / "public" / "icon-192.svg")
    (generated / "color" / "storage_logo_color_0256.png").copy(ui / "public" / "icon-256.svg")
    (generated / "color" / "storage_logo_color_0512.png").copy(ui / "public" / "icon-512.svg")
    (generated / "color" / "storage_logo_color_1024.png").copy(ui / "public" / "icon-1024.svg")
    (generated / "color" / "storage_logo_color.svg").copy(ui / "public" / "icon.svg")

