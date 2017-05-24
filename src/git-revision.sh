#!/bin/bash
revisioncount=$(git rev-list  `git rev-list --tags --no-walk --max-count=1`..HEAD --count)
projectversion=`git describe --tags --long`
cleanversion=${projectversion%%-*}

if [ -n "$(git status --porcelain . | grep -v ?? | grep -v src/mrbus)" ]
then
	revisioncount="$revisioncount+"
fi

echo "$cleanversion.$revisioncount"
