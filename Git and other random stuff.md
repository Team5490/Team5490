Basics of Git
============

Git is repository of our work, and also a way to preserve revision history. It allows us to share what we are working on in a way that has been optimised for coding, and allows us to revert to any previous version if Dave somehow screws it up.


To begin, if you have never used git before, go to https://github.com and create an account. After doing this, ask one of us (currently Joren, Tyler, Matt) to give you permission to access the repository.

In the meantime, there are a few BASH commands that you will need to know to connect to the git repository. These commands are run in the Terminal program that you can find using spotlight or by searching endlessly through your folders like an especially tech-savvy caveman (I have a tendency to do this).once terminal is running, the BASH commands you will need to know are:

	cd

This command stands for change directory, and changes what folder your terminal is working inside.

	ls

This command stands for list, and lists all the files and directories contained in your current directory (folder).

	pwd

This command stands for print working directory, and simply gives the name of the directory your terminal is currently in.

There are some other commands that are useful to know for various other things, but you'll pick them up as you need them. While in the terminal, type simply:

	git

If a window pops up suggesting you download X-Code Development Tools, follow the instructions and click the suggested buttons. The download should go fairly quickly, but is needed to use git in your command line.
Also, if you are ever confused about a command, you can type:

	help

followed by the command you’re unsure of (in one line).

Once you've been added as a contributor, create a folder somewhere on your computer where you want all the files from the project to go. The way git works is that even though everything is in our online repository, the files are saved locally on your computer, and all your changes remain local until you want to share them or back them up remotely. After you create the folder, navigate to it in the terminal window. Probably the easiest way to do this is to type cd into the terminal, and then click and drag your folder into the terminal window. This should copy the file location into the command line, leaving you with something like:

	cd /Users/student/Documents/Robotics2015

After which you can hit enter and to make sure it worked, type pwd. It should return the address of your folder (i.e. /Users/student/Documents/Robotics2015).

With your folder ready, you can enter the command:

	git clone https://github.com/Team5490/Team5490.git

which will copy the files that have already been created into your folder in a folder called Team5490. If prompted for username, type the username you used for your github account, and likewise for password. With the repository cloned onto your computer, you should be able to see, in that folder, everything that has been added to the repository.


From here, you can make changes to files, add new files, delete files, etc. in exactly the same way as you would with any normal folder. Whenever you have made any substantial changes, and especially if these are changes you want to save, or if you are for example finishing up for the night and want to back up your work, these are the steps to follow to add your changes to the repository. First you will need to use the git add command to tell git what you want it to track. Usually you will want it to track everything you have done. To do this, run any of these commands (they do the same thing):

	git add --a

	git add -A

	git add *

If for one reason you only want git to track some of the files you have added, you can add an individual file using:

	git add filename.ext

An easy way to see if you have any files to add is to use the command:

	git status

If you have a file or files to add you will see the message "Changes not staged for commit" followed by a list of these files. (You may also see “Untracked files”. If it contains any files that you have modified, you forgot to do git add.) After adding the files, the git status will instead say "Changes to commit", again followed by a list of 
files. To commit these files, run the command:

	git commit -m "a useful description of what you have done"

It is very important that the description of the commit describe what you did since your last commit, so that if we ever have to roll back to a previous version, we know what happened in each commit. From here, another git status will say "Your branch is ahead of origin master by 1 commit" in the best case, or may say something to the effect of "Your branch is not up to date." Either way, it is good to get into the habit of using

	git pull

somewhat frequently while working. What this does is update your files to match the changes that others have added to the repository, so that you don't end up with numerous conflicts. If you do receive the message that there is a merging conflict, meaning that two different changes have been made to the same file, you will have to manually merge, which we can go over when it comes up. After making sure you are up to date by using git pull, you can now run

	git push origin master

in order to add your work to the remote repository. After this, whenever anybody working on the project does a git pull, your work will be added to their local directory.
There is more to git than this, but for the most part, aside from merging, you won't be using much more than this. An example of the entire process would be:

	git pull

	git add -A

	**Note: ```git add *``` is getting deprecated. Don’t get used to it, use the -A tag.

	git commit -m "I did it! I used a bit of git!"

	git push origin master

Something the be careful of is that if you hit enter after typing

	git commit

without anything after, you will be taken to a command line text editor called vim. It will seem scary at first because your keys won't do what you expect them to, but it's pretty easy to get out and back to you happy bash world. Simply press escape to make sure you are in command mode, and then use either

	:q
or

	:x

Both will get you out of vim. If you really want to learn vim, it is explained pretty well at http://www.linux.com/learn/tutorials/228600-vim-101-a-beginners-guide-to-vim.


Some Other Small Things
---------

Although most of the work we will be doing is in java, there will be times when we have to edit text files as well. We all know that Text Edit is just about the most boring text editor out there. Even worse, it uses 8-space tabs. That's a lot of wasted space that looks bad, and what's more, Eclipse uses 4-space tabs. Being ugly is bad enough, but inconsistency is worse. However, there is is very nice text editor with lots of pretty color palates and syntax coloring(more useful if you get into javascript or python) called Sublime. It is subscription based, but the company offers an unlimited duration full-feature trial. Because stronk business model is stronk. But more importantly, it has 4-space tabs. This isn't that important, but really you won't want to use Text Edit after Sublime.

---

## Some Small Terminal Things ##

It can get annoying to have to constantly type out the whole path, and there are a couple ways to make life easier on your fingers. When you type out a file path, instead of typing out the full /Users/[user]/foo/, you can use a tilde (~) at the start of the file path. The tilde goes at the start of the file path and replaces /Users/[user]/ in the start of the file path. For example, a file path of ~/Documents/ takes you to the current user’s documents folder.

If you’re messing around in the terminal and you get stuck with a program running that you don’t want or just want to escape from what you’re running, press control + c.

---

About .md

You probably noticed this is actually formatted. There is a filetype called markdown (extension: .md) which allows near-plaintext with formatting that doesn’t do what .txt does (If you're familiar with reddit formatting, it's prett much the same). Of course, this doesn’t mean that we shouldn’t use .txt, which is easier and more efficient where format doesn’t matter. .md files can also be edited through the web gui on github.com, by opening the file and clicking the pencil.
For info on the markdown, look no further than [here](https://github.com/fletcher/MultiMarkdown/blob/master/Documentation/Markdown%20Syntax.md)
