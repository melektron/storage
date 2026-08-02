"""
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
02.08.26, 19:54

inkmark pipeline to generate storage logo variants
"""

import inkmark

if __name__ == "__main__":
    base_collection = inkmark.prepare(
        __file__,
        "storage_logo",
        input_file="storage_logo.inkscape.svg"
    )

    # full color version
    color = base_collection.sub_collection("color")
    # monochrome black version
    black = base_collection.sub_collection("black", "assume")
    # monochrome white version
    white = base_collection.sub_collection("white", "assume")
    # monochrome colorless version that can be colored using "fill" attribute on svg element
    colorless = base_collection.sub_collection("colorless", "assume")

    # generate the monochromatic inkscape base images by 
    # subtracting the inner elements from the outer elements
    base_collection.run_inkscape_actions(

        "select-all",
        "object-set-property:stroke,#000000",
        f"export-filename:{black.input_file}",
        "export-do",
        "select-clear",

        "select-all",
        "object-set-property:stroke,#ffffff",
        f"export-filename:{white.input_file}",
        "export-do",
        "select-clear",

        "select-all",
        "object-set-property:stroke,auto",
        # convert green lines to filled path so they can be
        # recolored using "fill" (independently from box lines)
        "select-all",
        "select-by-id:codeline1",
        "select-by-id:codeline2",
        "select-by-id:codeline3",
        "select-by-id:codeline4",
        "select-by-id:codeline5",
        "object-stroke-to-path",
        f"export-filename:{colorless.input_file}",
        "export-do",

        "file-close",
    )

    # generate different formats in subdirectories for each color variant

    color.generate_standard_formats("page")
    color.generate_special_formats(
        "black", "white",
        # temporarily set box stroke to white when exporting with dark background
        pre_dark_mode_actions=[
            "select-by-id:box1",
            "select-by-id:box2",
            "select-by-id:box3",
            "object-set-property:stroke,#ffffff",
        ],
        post_dark_mode_actions=[
            "object-set-property:stroke,#000000",
        ]
    )

    black.generate_standard_formats("page")
    black.generate_special_formats("white", "white")

    white.generate_standard_formats("page")
    white.generate_special_formats("black", "black")

    # colorless variant only makes sense for SVGs as only they can be 
    # recolored using fill attribute. For these, we don't do separate subdirs.
    colorless.generate_standard_formats("page", svg_only=True)
