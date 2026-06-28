# https://nixos.wiki/wiki/Node.js
{ pkgs ? import <nixpkgs> {} }:

let
in pkgs.mkShell {
  packages = with pkgs; [
    nodejs
    #nodePackages.npm

    # needed for native tls variants of sea orm (including for sea-orm-cli)
    pkg-config
    openssl
  ];

  #inherit NPM_CONFIG_PREFIX;

  #shellHook = ''
  #  export PATH="${NPM_CONFIG_PREFIX}/bin:$PATH"
  #'';
  shellHook = ''
    export PATH="$HOME/.cargo/bin:$PATH"
  '';
}
