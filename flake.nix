{
  description = "storage_client_test";

  inputs = {
    nixpkgs.url = "nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";

  };

  outputs = { self, nixpkgs, flake-utils, ... }@inputs:
    nixpkgs.lib.recursiveUpdate (flake-utils.lib.eachDefaultSystem (system: 
      let
        pkgs = import nixpkgs { inherit system; };
      in
      {
        # put stuff here that runs for every platform
        # like e.g. your package
        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            pkg-config # this being in pkg-config apparently isn't enough for rust analyzer

            # python for brandkit pipeline
            python314
          ];

          nativeBuildInputs = with pkgs; [
            gcc
            cargo
            rustc
            dioxus-cli

            # necessary for web build
            lld
            binaryen  # includes wasm-opt
            wasm-bindgen-cli_0_2_126
            
            # additionally necessary for desktop build
            pkg-config
            wrapGAppsHook4
          ];

          buildInputs = with pkgs; [
            webkitgtk_4_1
            #gtk3
            #alsa-lib
            #libudev-devd
            xdotool

            openssl
            #atk
          ];


          shellHook = ''
            export PATH="$HOME/.cargo/bin:$PATH"
          '';
        };
      }
    )) {};
}