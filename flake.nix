{
  description = "Double pendulum";

  inputs = { nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable"; };

  outputs = { self, nixpkgs }:
    let forAllSystems = nixpkgs.lib.genAttrs nixpkgs.lib.systems.flakeExposed;
    in {
      packages = forAllSystems (system: rec {
        default = dpend;
        dpend = nixpkgs.legacyPackages.${system}.stdenv.mkDerivation {
          name = "dpend";
          version = "1.0.0";
          src = ./src;
          buildInputs = [ ];
          buildPhase = "gcc -lm -g -Wall -Werror main.c display.c sim.c util.c -o dpend";
          installPhase = ''
            mkdir -p $out/bin
            cp dpend $out/bin
          '';
        };
      });
    };
}
