/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace ORB_SLAM2
{

class KeyFrame;
class Frame;

/* Recognition Database
 * The database contains an inverted index that stores for
 * each visual word in the vocabulary in which keyframes it has
 * been seen, and its weight in the keyframe. Therefore querying
 * the database can be done very efficiently by checking only
 * common visual words. New keyframes are inserted in the
 * database by the loop closer task (see Fig. 1), after the local
 * mapping has finished processing it.
 *
 * When the relocalisation or the loop detector query the
 * database, a similarity score is computed between all those
 * keyframes that share visual words with the query image.
 * Because there exists visual overlap between keyframes, there
 * will not exist a unique keyframe with a high score, rather
 * there will be several keyframes with high scores.
 *
 * we form covisibility groups which sum the
 * scores of all those keyframes that are directly connected in the
 * covisibility graph. The one with the highest individual score,
 * being the keyframe match. Instead of getting just the best match,
 * we get all those matches whose scores are higher than the 75% of
 * the best score.*/
class KeyFrameDatabase
{
public:

    KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
