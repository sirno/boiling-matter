{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    { nixpkgs, ... }:
    let
      system = "aarch64-darwin";
      pkgs = nixpkgs.legacyPackages.${system};
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          dfu-util
          ccache
          cmake
          ninja
          python3
        ];

        IDF_CCACHE_ENABLE = 1;

        shellHook = ''
          . esp-idf/export.sh
          . esp-matter/export.sh
        '';
      };
    };
}
