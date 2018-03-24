#!/bin/bash
revisioncount=$(git rev-list  `git rev-list --tags="[0-9][.][0-9]" --no-walk --max-count=1`..HEAD --count)
projectversion=`git describe --tags --match "[0-9][.][0-9]" --long`
cleanversion=${projectversion%%-*}

if [ -n "$(git status --porcelain . | grep -v ?? | grep -v src/mrbus)" ]
then
	revisioncount="$revisioncount+"
fi

echo "$cleanversion.$revisioncount"
