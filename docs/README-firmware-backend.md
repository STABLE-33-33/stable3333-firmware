# Backend gratuit des mises à jour firmware

Cette solution utilise uniquement GitHub gratuit pour les dépôts publics:

- GitHub Pages publie `docs/firmware/manifest.json`.
- GitHub Releases héberge les fichiers `.bin`.
- GitHub Actions compile le `.ino`, calcule le SHA-256 et met à jour le manifest.
- `docs/admin/index.html` sert de panneau administrateur statique.

## Mise en place

1. Créer un dépôt GitHub public, par exemple `stable3333-firmware`.
2. Pousser ce dossier `docs/`, le dossier `firmware/` et `.github/workflows/firmware-release.yml`.
3. Dans GitHub, activer Pages avec la source `Deploy from a branch`, branche `main`, dossier `/docs`.
4. Dans l'application iOS, utiliser l'URL:

   `https://<owner>.github.io/<repo>/firmware/manifest.json`

5. Créer un token GitHub finement limité avec accès au dépôt:
   - Contents: Read and write
   - Actions: Read and write
   - Metadata: Read-only

## Publication admin

1. Ouvrir `https://<owner>.github.io/<repo>/admin/`.
2. Entrer owner, repo, branche et token.
3. Importer le fichier `.ino`.
4. Choisir `beta` ou `final`.
5. Cliquer `Propulser la version`.
6. Attendre que GitHub Actions termine.

Quand le workflow réussit, l'application détecte automatiquement la nouvelle version via le manifest public.
