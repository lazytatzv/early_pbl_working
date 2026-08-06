{
  description = "dev shell";

  inputs.nixpkgs.url =
    "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { nixpkgs, ... }:
  let
    system = "x86_64-darwin";

    pkgs = import nixpkgs {
      inherit system;
    };
  in {
    devShells.${system}.default =
      pkgs.mkShell {
        packages = with pkgs; [
          uv
        ];
      };
  };
}
