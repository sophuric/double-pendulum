{
  description = "Double pendulum";

  inputs = { nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable"; };

  outputs = { self, nixpkgs }:
    let
      forAllSystems = nixpkgs.lib.genAttrs nixpkgs.lib.systems.flakeExposed;
      buildInputs = system:
        with nixpkgs.legacyPackages.${system}; [
          (symengine.overrideAttrs (prev: {
            buildInputs = prev.buildInputs ++ [ libllvm ];
            cmakeFlags = prev.cmakeFlags
              ++ [ (lib.cmakeBool "WITH_LLVM" true) ];
          }))
          gmp
          mpfr
        ];
    in {
      devShells = forAllSystems (system: rec {
        default = nixpkgs.legacyPackages.${system}.mkShell {
          buildInputs = buildInputs system;
        };
      });
      packages = forAllSystems (system: rec {
        default = self.packages.${system}.dpend;
        dpend = nixpkgs.legacyPackages.${system}.stdenv.mkDerivation {
          name = "dpend";
          version = "1.0.0";
          src = ./.;
          buildInputs = buildInputs system;
          buildPhase = "bash ./build release";
          installPhase = ''
            mkdir -p $out/bin
            cp out/dpend $out/bin
          '';
        };
      });
    };
}
