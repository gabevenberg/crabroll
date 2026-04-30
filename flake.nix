{
  description = "Crabroll rust development flake.";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = {
    nixpkgs,
    rust-overlay,
    treefmt-nix,
    ...
  }: let
    forAllSystems = function:
      nixpkgs.lib.genAttrs [
        "x86_64-linux"
        "aarch64-linux"
      ] (system:
        function {
          pkgs = (
            import nixpkgs {
              overlays = [(import rust-overlay)];
              inherit system;
            }
          );
          inherit system;
        });
    treefmtEval = forAllSystems ({pkgs, ...}: treefmt-nix.lib.evalModule pkgs ./treefmt.nix);
  in {
    formatter = forAllSystems ({system, ...}: treefmtEval.${system}.config.build.wrapper);

    devShells =
      forAllSystems
      ({pkgs, ...}: {
        default = pkgs.mkShell {
          buildInputs = with pkgs; [
            probe-rs-tools
            (rust-bin.fromRustupToolchainFile ./rust-toolchain.toml)
            typst
          ];
        };
      });
  };
}
