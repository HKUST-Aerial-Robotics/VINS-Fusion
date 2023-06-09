# Git workflow

1. Create a new branch to add/fix a feature/bug

```
git checkout -b <name>/add_<feature_name>
```

2. Push the branch to the remote (i.e. upstream) repository

```
# On the branch
git push -u origin <branch_name>
```

3. Work on this branch and push

```
# On this working branch
git push
```

4. If this branch falls behind the main branch, use `rebase` to move this
   branch's changes on top of the latest `main`

```
# On this working branch
git rebase origin/main
```

5. After finishing the code on this branch, merge it to the `main` branch

```
git checkout main
# On the main branch
git merge --no-ff <branch_name>
```



