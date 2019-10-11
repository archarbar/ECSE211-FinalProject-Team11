# ECSE211 Ballistic Launcher Project

This is your repository for the fifth lab as well as the project.

## Source files

You must place your source files in the 
`src/ca/mcgill/ecse211/project/` folder, where `Main.java` is. Your source files
must all have the package declaration 

```java
package ca.mcgill.ecse211.project;
```

**Do not modify the other source files in this repository without written permission!**

They are meant to help us (and you) with grading and reviewing the code. You can see the results
of `checkstyle` and SpotBugs at

travis-ci.com/mcgill-ecse211-f19/project-ecse211_gXX

where XX is your group number. 

You may also run these locally after cloning the project and pulling the latest changes.

Run `checkstyle` locally before you commit by running

```bash
./gradlew checkstyle
```

on your local machine from the repo root directory.

You can run SpotBugs locally using the Eclipse
[plugin](https://marketplace.eclipse.org/content/spotbugs-eclipse-plugin).

## Documentation

Write your documentation in the [`docs`](docs) folder. **Do NOT include any personal information, such as your student number, since GitHub Pages is publicly viewable!**

If it is not already, enable GitHub Pages for your repository following [these steps](https://help.github.com/en/articles/configuring-a-publishing-source-for-your-github-pages-site#choosing-a-publishing-source).

To generate Javadoc, right-click the project and select `Export > Javadoc`. Change the output location to end in `docs/javadoc` instead of `doc`. This will make your Javadoc viewable directly on GitHub Pages.


## Releases

From time to time (about once a week), you will be asked to create a release that represents your progress so far. A release is simply a tagged git commit. To do that, go to Releases:

github.com/mcgill-ecse211-f19/project-ecse211_gXX/releases

(Replace XX with your team number)

Then select Draft a new release, and give your release an appropriate tag, name and description, eg:

**Tag version:** v1.0.0 

#### Version 1.0

Week 1 documents and software. Changelog:
- Add better timed sampling
- Add hardware design alternative sketches
- Fix localization bug 


## Connecting repo to Eclipse

For more information on connecting this repository to your local Eclipse installation,
please refer to the [Clean Code Tutorial](https://mcgill-ecse211-f19.github.io/getting_started_guide/CleanCodeTutorial-F19#step-by-step-instructions)
(modify the steps for the project as needed).
